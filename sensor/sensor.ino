#include <RadioLib.h>
#include <Wire.h>
#include <AHT20.h>
#include <DallasTemperature.h>
#include <BH1750.h>
#include <OneWire.h>

#define I2C_SDA_PIN PB7
#define I2C_SCL_PIN PB6
#define ONE_WIRE_BUS PB0
#define SOIL_MOISTURE_PIN PA1

static const float FREQ_MHZ = 923.2f;  // JP band example
static const float BW_KHZ = 500.0f;
static const uint8_t SF = 5;
static const uint8_t CR = 5;  // 4/5
static const uint8_t SYNCWORD = 0x12;
static const int8_t PWR_DBM = 22;
static const uint16_t PREAMBLE = 8;
static const float TCXO_V = 1.6f;
static const bool USE_LDO = false;

// Mesh / Node Config
static const int DEVICE_ID = 5;  // rank-1 sensor node
static const uint8_t MAX_TTL = 3;

static const unsigned long DISCOVERY_SCAN_INTERVAL = 5000UL;      // 5s log
static const unsigned long BEACON_COLLECTION_TIME = 30000UL;      // 30s window
static const unsigned long BEACON_TIMEOUT = 60000UL;              // 60s stale
static const unsigned long REG_PING_TIMEOUT = 3000UL;             // 3s wait
static const unsigned long ACK_TIMEOUT = 3000UL;                  // 3s wait
static const unsigned long UPLINK_INTERVAL = 30000UL;             // 30s data

static const uint8_t MAX_DATA_RETRIES = 3;
static const uint8_t MAX_FLOOD_RETRIES = 2;

static const int RSSI_THRESHOLD = -115;          // dBm
static const int RANK_PREFERENCE_WEIGHT = 10;    // dB per hop
static const uint8_t MAX_NEIGHBORS = 10;         // neighbor table size

STM32WLx_Module wl;
STM32WLx radio(&wl);

// RF switch table (PA4 / PA5) - mandatory for Wio-E5 Mini
static const uint32_t RFSW_PINS[] = {
  PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};

static const Module::RfSwitchMode_t RFSW_TABLE[] = {
  { STM32WLx::MODE_IDLE,   { LOW,  LOW  } },
  { STM32WLx::MODE_RX,     { HIGH, LOW  } },
  { STM32WLx::MODE_TX_LP,  { LOW,  HIGH } },
  { STM32WLx::MODE_TX_HP,  { LOW,  HIGH } },
  END_OF_MODE_TABLE,
};

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
AHT20 aht20;
BH1750 lightMeter;

enum SensorState {
  DISCOVERING,
  WAIT_REG_PING,
  JOINED
};

struct Neighbor {
  int id = -1;
  uint8_t rank = 255;
  int rssi = -127;
  unsigned long lastHeard = 0;
  bool isValid = false;
};

struct MeshMessage {
  bool valid = false;

  String type;

  int src = -1;
  bool hasSrc = false;

  int dst = -1;
  bool hasDst = false;

  int seq = 0;
  bool hasSeq = false;

  int ttl = 0;
  bool hasTtl = false;

  int rank = 0;
  bool hasRank = false;

  int ack = 0;
  bool hasAck = false;

  int orig = -1;
  bool hasOrig = false;

  String data;
  bool hasData = false;
};

struct UplinkState {
  bool active = false;
  uint16_t seq = 0;
  String payload;
  uint8_t retries = 0;
  unsigned long lastSend = 0;
  bool floodMode = false;
};

SensorState currentState = DISCOVERING;
Neighbor neighborTable[MAX_NEIGHBORS];

int bestRepeaterId = -1;
uint8_t bestRepeaterRank = 255;

unsigned long bootTime = 0;
unsigned long lastBeaconHeard = 0;
unsigned long lastDiscoveryLog = 0;
unsigned long lastAnnounceTime = 0;
unsigned long lastUplinkTime = 0;

uint16_t uplinkSequence = 0;
uint16_t controlSequence = 0;

UplinkState uplinkState;

uint16_t nextUplinkSeq() {
  return ++uplinkSequence;
}

uint16_t nextControlSeq() {
  return ++controlSequence;
}

void clearNeighborTable() {
  for (uint8_t i = 0; i < MAX_NEIGHBORS; i++) {
    neighborTable[i] = Neighbor();
  }
}

Neighbor* findNeighbor(int id) {
  for (uint8_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighborTable[i].isValid && neighborTable[i].id == id) {
      return &neighborTable[i];
    }
  }
  return nullptr;
}

Neighbor* findEmptySlot() {
  for (uint8_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (!neighborTable[i].isValid) {
      return &neighborTable[i];
    }
  }
  return nullptr;
}

void updateNeighbor(int nodeId, uint8_t rank, int rssi, unsigned long now) {
  Neighbor* slot = findNeighbor(nodeId);
  if (!slot) {
    slot = findEmptySlot();
  }

  if (!slot) {
    Serial.println(F("[NEIGHBOR] Table full"));
    return;
  }

  slot->id = nodeId;
  slot->rank = rank;
  slot->rssi = rssi;
  slot->lastHeard = now;
  slot->isValid = true;
}

void pruneStaleNeighbors(unsigned long now) {
  for (uint8_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighborTable[i].isValid && (now - neighborTable[i].lastHeard) > BEACON_TIMEOUT) {
      Serial.print(F("[NEIGHBOR] Timeout -> "));
      Serial.println(neighborTable[i].id);
      neighborTable[i].isValid = false;
    }
  }
}

void markNeighborUnreliable(int nodeId) {
  Neighbor* n = findNeighbor(nodeId);
  if (n) {
    n->isValid = false;
    Serial.print(F("[NEIGHBOR] Mark unreliable "));
    Serial.println(nodeId);
  }
}

uint8_t countValidNeighbors() {
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighborTable[i].isValid && neighborTable[i].rank < 255) {
      cnt++;
    }
  }
  return cnt;
}

String getSensorReadings() {
  // DS18B20 soil temp
  sensors.requestTemperatures();
  float soilTemp = sensors.getTempCByIndex(0);

  // Quick retry loop for 85.0°C "not ready"
  for (int i = 0; soilTemp == 85.0f && i < 100; i++) {
    delay(1);
    soilTemp = sensors.getTempCByIndex(0);
  }

  // AHT20 air temp / humidity
  float airTemp = aht20.getTemperature();
  float humidity = aht20.getHumidity();

  // BH1750 lux
  float lux = lightMeter.readLightLevel();

  // Soil moisture (raw -> %)
  int rawMoisture = analogRead(SOIL_MOISTURE_PIN);
  int moisturePercent = map(rawMoisture, 0, 4095, 0, 100);

  char msg[128];
  snprintf(
    msg,
    sizeof(msg),
    "ID:%d,ST:%.2f,AT:%.2f,H:%.2f,L:%.2f,M:%d",
    DEVICE_ID,
    soilTemp,
    airTemp,
    humidity,
    lux,
    moisturePercent
  );

  return String(msg);
}

String buildMessage(const char* type, int src, int dst, int seq, int rank = -999, int ttl  = -999, int ack  = -999, const String& data = "") {

  String msg = "TYPE=";
  msg += type;
  msg += ";SRC=";
  msg += src;
  msg += ";DST=";
  msg += dst;
  msg += ";SEQ=";
  msg += seq;

  if (rank != -999) {
    msg += ";RANK=";
    msg += rank;
  }

  if (ttl != -999) {
    msg += ";TTL=";
    msg += ttl;
  }

  if (ack != -999) {
    msg += ";ACK=";
    msg += ack;
  }

  if (data.length() > 0) {
    msg += ";DATA=";
    msg += data;
  }

  return msg;
}

MeshMessage parseMessage(const String& raw) {
  MeshMessage msg;

  String header = raw;
  int dataIdx = raw.indexOf(";DATA=");

  if (dataIdx >= 0) {
    header = raw.substring(0, dataIdx);
    msg.data = raw.substring(dataIdx + 6);
    msg.hasData = true;
  } else if (raw.startsWith("DATA=")) {
    msg.data = raw.substring(5);
    msg.hasData = true;
    header = "";
  }

  int start = 0;
  while (start < header.length()) {
    int end = header.indexOf(';', start);
    if (end < 0) {
      end = header.length();
    }

    String token = header.substring(start, end);
    int eqIdx = token.indexOf('=');

    if (eqIdx > 0) {
      String key = token.substring(0, eqIdx);
      String val = token.substring(eqIdx + 1);

      if (key == "TYPE") {
        msg.type = val;
      } else if (key == "SRC") {
        msg.src = val.toInt();
        msg.hasSrc = true;
      } else if (key == "DST") {
        msg.dst = val.toInt();
        msg.hasDst = true;
      } else if (key == "SEQ") {
        msg.seq = val.toInt();
        msg.hasSeq = true;
      } else if (key == "TTL") {
        msg.ttl = val.toInt();
        msg.hasTtl = true;
      } else if (key == "RANK") {
        msg.rank = val.toInt();
        msg.hasRank = true;
      } else if (key == "ACK") {
        msg.ack = val.toInt();
        msg.hasAck = true;
      } else if (key == "ORIG") {
        msg.orig = val.toInt();
        msg.hasOrig = true;
      }
    }

    start = end + 1;
  }

  if (msg.type.length() > 0 && msg.hasSrc && msg.hasDst) {
    msg.valid = true;
  }

  return msg;
}

void sendMessage(const String& payload) {
  int16_t state = radio.transmit(payload);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print(F("[TX] "));
    Serial.println(payload);
  } else {
    Serial.print(F("[TX][ERR] "));
    Serial.println(state);
  }
}

void announceToRepeater() {
  if (bestRepeaterId < 0) return;

  String message = buildMessage(
    "ANNOUNCE",
    DEVICE_ID,
    bestRepeaterId,
    nextControlSeq(),
    bestRepeaterRank + 1
  );

  sendMessage(message);
  lastAnnounceTime = millis();

  Serial.print(F("[JOIN] Announce -> repeater "));
  Serial.println(bestRepeaterId);
}

void sendJoinConfirm() {
  String confirm = buildMessage(
    "JOIN_CONFIRM",
    DEVICE_ID,
    bestRepeaterId,
    nextControlSeq()
  );
  sendMessage(confirm);

  currentState = JOINED;
  uplinkState.active = false;
  lastUplinkTime = millis();

  Serial.print(F("[JOIN] Joined via repeater "));
  Serial.println(bestRepeaterId);
}

bool hasCollectedEnoughBeacons(unsigned long now) {
  if ((now - bootTime) < BEACON_COLLECTION_TIME) {
    return false;
  }

  for (uint8_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighborTable[i].isValid && neighborTable[i].rank == 1) {
      if (neighborTable[i].rssi > RSSI_THRESHOLD + 10) {
        return true;
      }
    }
  }

  if (countValidNeighbors() >= 2) {
    return true;
  }

  return false;
}

int selectBestRepeater() {
  Neighbor* best = nullptr;
  int bestScore = -999999;

  for (uint8_t i = 0; i < MAX_NEIGHBORS; i++) {
    Neighbor& n = neighborTable[i];
    if (!n.isValid) continue;
    if (n.rank >= 255) continue;
    if (n.rssi < RSSI_THRESHOLD) continue;

    int score = n.rssi - (n.rank * RANK_PREFERENCE_WEIGHT);
    if (score > bestScore) {
      bestScore = score;
      best = &n;
    }
  }

  if (best) {
    bestRepeaterRank = best->rank;
    return best->id;
  }

  return -1;
}

bool isRepeaterStillAlive(int repeaterId, unsigned long now) {
  if (repeaterId == -1) return false;
  Neighbor* n = findNeighbor(repeaterId);
  if (!n || !n->isValid) return false;
  if ((now - n->lastHeard) > BEACON_TIMEOUT) return false;
  return true;
}

void attemptRepeaterFailover(unsigned long now) {
  Serial.println(F("[FAILOVER] Attempting backup repeater"));
  pruneStaleNeighbors(now);

  int newRepeater = selectBestRepeater();
  if (newRepeater != -1 && newRepeater != bestRepeaterId) {
    bestRepeaterId = newRepeater;
    Serial.print(F("[FAILOVER] Switching to repeater "));
    Serial.println(bestRepeaterId);

    announceToRepeater();
    currentState = WAIT_REG_PING;
    uplinkState.active = false;
  } else {
    Serial.println(F("[FAILOVER] No backup found, restarting discovery"));
    bestRepeaterId = -1;
    currentState = DISCOVERING;
    uplinkState.active = false;
    clearNeighborTable();
  }
}

void handleDiscoveryState(unsigned long now) {
  pruneStaleNeighbors(now);

  if (now - lastDiscoveryLog > DISCOVERY_SCAN_INTERVAL) {
    Serial.print(F("[DISCOVERY] Scanning... neighbors="));
    Serial.println(countValidNeighbors());
    lastDiscoveryLog = now;
  }

  if (hasCollectedEnoughBeacons(now)) {
    bestRepeaterId = selectBestRepeater();
    if (bestRepeaterId != -1) {
      Serial.print(F("[DISCOVERY] Selected repeater "));
      Serial.println(bestRepeaterId);
      announceToRepeater();
      currentState = WAIT_REG_PING;
    } else {
      Serial.println(F("[DISCOVERY] No suitable repeater yet"));
    }
  }
}

void handleWaitRegPingState(unsigned long now) {
  if ((now - lastAnnounceTime) > REG_PING_TIMEOUT) {
    Serial.println(F("[JOIN] REG_PING timeout, restarting discovery"));
    markNeighborUnreliable(bestRepeaterId);
    bestRepeaterId = -1;
    currentState = DISCOVERING;
  }
}

void startNewUplink() {
  uplinkState.active = true;
  uplinkState.seq = nextUplinkSeq();
  uplinkState.payload = getSensorReadings();
  uplinkState.retries = 0;
  uplinkState.floodMode = false;

  Serial.print(F("[UPLINK] Start seq="));
  Serial.println(uplinkState.seq);
}

void transmitUplink() {
  if (!uplinkState.active) return;

  String msg = buildMessage(
    uplinkState.floodMode ? "FLOOD" : "DATA",
    DEVICE_ID,
    bestRepeaterId,
    uplinkState.seq,
    -999,
    uplinkState.floodMode ? MAX_TTL : 1,
    -999,
    uplinkState.payload
  );

  sendMessage(msg);
  uplinkState.lastSend = millis();
  uplinkState.retries++;

  Serial.print(F("[UPLINK] Sent mode="));
  Serial.print(uplinkState.floodMode ? "FLOOD" : "DATA");
  Serial.print(F(" seq="));
  Serial.print(uplinkState.seq);
  Serial.print(F(" retry="));
  Serial.println(uplinkState.retries);
}

void handleUplinkTimeout(unsigned long now) {
  if (!uplinkState.active) return;

  if (!uplinkState.floodMode) {
    if (uplinkState.retries < MAX_DATA_RETRIES) {
      Serial.println(F("[UPLINK] No ACK, retry DATA"));
      transmitUplink();
    } else {
      Serial.println(F("[UPLINK] Escalate to FLOOD"));
      uplinkState.floodMode = true;
      uplinkState.retries = 0;
      transmitUplink();
    }
  } else {
    if (uplinkState.retries < MAX_FLOOD_RETRIES) {
      Serial.println(F("[UPLINK] No ACK, retry FLOOD"));
      transmitUplink();
    } else {
      Serial.println(F("[UPLINK] FLOOD exhausted, restart discovery"));
      uplinkState.active = false;
      currentState = DISCOVERING;
      bestRepeaterId = -1;
      clearNeighborTable();
    }
  }
}

void handleAck(const MeshMessage& msg) {
  if (!uplinkState.active || !msg.hasAck) return;
  if (msg.ack != uplinkState.seq) return;
  if (msg.src != bestRepeaterId) return;

  Serial.print(F("[ACK] Received for seq="));
  Serial.println(uplinkState.seq);

  uplinkState.active = false;
  uplinkState.floodMode = false;
  Serial.println(F("[UPLINK] Success"));
}

void handleJoinedState(unsigned long now) {
  if (!uplinkState.active) {
    if ((now - lastUplinkTime) >= UPLINK_INTERVAL) {
      startNewUplink();
      transmitUplink();
      lastUplinkTime = now;
    }
  }

  if (uplinkState.active && (now - uplinkState.lastSend) > ACK_TIMEOUT) {
    handleUplinkTimeout(now);
  }

  pruneStaleNeighbors(now);

  if (!isRepeaterStillAlive(bestRepeaterId, now)) {
    Serial.println(F("[KEEPALIVE] Repeater lost"));
    attemptRepeaterFailover(now);
  }
}

void handleBeacon(const MeshMessage& msg, int rssi, unsigned long now) {
  if (!msg.hasRank || msg.rank >= 255) return;

  updateNeighbor(msg.src, msg.rank, rssi, now);
  Serial.print(F("[BEACON] From="));
  Serial.print(msg.src);
  Serial.print(F(" rank="));
  Serial.print(msg.rank);
  Serial.print(F(" rssi="));
  Serial.println(rssi);

  if (currentState == JOINED && msg.src == bestRepeaterId) {
    lastBeaconHeard = now;
    Serial.println(F("[KEEPALIVE] Beacon from repeater"));
  }
}

void handleRegPing(const MeshMessage& msg) {
  if (currentState != WAIT_REG_PING) return;
  if (msg.src != bestRepeaterId) {
    Serial.print(F("[JOIN] Unexpected REG_PING from "));
    Serial.println(msg.src);
    return;
  }

  Serial.print(F("[JOIN] REG_PING from repeater "));
  Serial.println(bestRepeaterId);
  sendJoinConfirm();
}

void processIncoming(const MeshMessage& msg, int rssi, unsigned long now) {
  if (!msg.valid) return;

  if (msg.dst != DEVICE_ID && msg.dst != -1) return;

  if (msg.type == "BEACON") {
    handleBeacon(msg, rssi, now);
  } else if (msg.type == "REG_PING") {
    handleRegPing(msg);
  } else if (msg.type == "ACK") {
    handleAck(msg);
  }
}

void pollRadio() {
  for (int i = 0; i < 3; i++) {
    String incoming;
    int16_t rxState = radio.receive(incoming);

    if (rxState == RADIOLIB_ERR_NONE) {
      int rssi = (int)radio.getRSSI();
      MeshMessage msg = parseMessage(incoming);
      if (msg.valid) {
        Serial.print(F("[RX] "));
        Serial.println(incoming);
      }
      processIncoming(msg, rssi, millis());
    } else if (rxState == RADIOLIB_ERR_RX_TIMEOUT || rxState == RADIOLIB_ERR_CRC_MISMATCH) {
      break;
    } else {
      Serial.print(F("[RX][ERR] "));
      Serial.println(rxState);
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println();
  Serial.println(F("=== Sensor Mesh Node ==="));

  bootTime = millis();

  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  Serial.println(F("[INIT] Sensors..."));
  if (!aht20.begin()) {
    Serial.println(F("[INIT][WARN] AHT20 init failed"));
  }

  sensors.begin();

  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println(F("[INIT][WARN] BH1750 init failed"));
  }

  radio.setRfSwitchTable(RFSW_PINS, RFSW_TABLE);
  Serial.println(F("[INIT] RF switch table applied"));

  int16_t stateInit = radio.begin(
    FREQ_MHZ,
    BW_KHZ,
    SF,
    CR,
    SYNCWORD,
    PWR_DBM,
    PREAMBLE,
    TCXO_V,
    USE_LDO
  );

  Serial.print(F("[INIT] radio.begin -> "));
  Serial.println(stateInit);

  if (stateInit != RADIOLIB_ERR_NONE) {
    Serial.println(F("[FATAL] LoRa init failed"));
    while (true) {
      delay(1000);
    }
  }

  radio.setCRC(true);
  clearNeighborTable();
  uplinkState = UplinkState();

  Serial.println(F("[INIT] Discovery started"));
}

void loop() {
  unsigned long now = millis();

  pollRadio();

  switch (currentState) {
    case DISCOVERING:
      handleDiscoveryState(now);
      break;

    case WAIT_REG_PING:
      handleWaitRegPingState(now);
      break;

    case JOINED:
      handleJoinedState(now);
      break;
  }

  delay(50);
}
