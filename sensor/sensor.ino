#include <RadioLib.h>
#include <Wire.h>
#include <AHT20.h>
#include <DallasTemperature.h>
#include <BH1750.h>
#include <OneWire.h>

// ======================= USER CONFIG =======================

// Pins
#define I2C_SDA_PIN PB7
#define I2C_SCL_PIN PB6
#define ONE_WIRE_BUS PB0
#define SOIL_MOISTURE_PIN PA1

// LoRa Parameters (FAST / SHORT RANGE)
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
static const int DEVICE_ID = 5;                         // rank-1 sensor node
static const uint8_t MAX_TTL = 3;                       // max hops for flood
static const unsigned long SEND_INTERVAL_MS = 30000UL;  // sensor uplink period
static const unsigned long SCAN_INTERVAL_MS = 5000UL;   // beacon log interval
static const unsigned long REG_PING_TIMEOUT_MS = 2000UL;
static const unsigned long ACK_TIMEOUT_MS = 3000UL;
static const unsigned long BEACON_LOSS_TIMEOUT_MS = 300000UL;  // 5 minutes
static const uint8_t MAX_UPLINK_RETRIES = 3;
static const uint8_t MAX_FLOOD_RETRIES = 2;

// ======================= RADIO OBJECTS =======================

STM32WLx_Module wl;
STM32WLx radio(&wl);

// RF switch table (PA4 / PA5) - mandatory for Wio-E5 Mini
static const uint32_t RFSW_PINS[] = {
  PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};

static const Module::RfSwitchMode_t RFSW_TABLE[] = {
  { STM32WLx::MODE_IDLE, { LOW, LOW } },
  { STM32WLx::MODE_RX, { HIGH, LOW } },
  { STM32WLx::MODE_TX_LP, { LOW, HIGH } },
  { STM32WLx::MODE_TX_HP, { LOW, HIGH } },
  END_OF_MODE_TABLE,
};

// ======================= SENSOR OBJECTS =======================

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
AHT20 aht20;
BH1750 lightMeter;

// ======================= STATE TYPES =======================

enum SensorState {
  WAIT_BEACON,
  WAIT_REG_PING,
  JOINED
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

struct PendingUplink {
  bool waiting = false;
  bool floodMode = false;
  uint16_t seq = 0;
  uint8_t retries = 0;
  unsigned long lastAttempt = 0;
  String payload;
};

// ======================= STATE VARS =======================

SensorState state = WAIT_BEACON;
int repeaterId = -1;
unsigned long lastBeaconHeard = 0;
unsigned long lastAnnounceAt = 0;
unsigned long lastScanLog = 0;
unsigned long lastSendMillis = 0;

uint16_t uplinkSequence = 0;
uint16_t controlSequence = 0;

PendingUplink uplink;

// ======================= HELPERS =======================

uint16_t nextUplinkSeq() {
  return ++uplinkSequence;
}

uint16_t nextControlSeq() {
  return ++controlSequence;
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
    moisturePercent);

  return String(msg);
}

// ---------- Message Builders ----------

String buildBaseMessage(const char* type, int src, int dst, int seq) {
  String msg = "TYPE=";
  msg += type;
  msg += ";SRC=";
  msg += src;
  msg += ";DST=";
  msg += dst;
  msg += ";SEQ=";
  msg += seq;
  return msg;
}

String buildAnnounceMessage() {
  int seq = nextControlSeq();
  String msg = buildBaseMessage("ANNOUNCE", DEVICE_ID, repeaterId, seq);
  msg += ";RANK=1";
  return msg;
}

String buildJoinConfirmMessage() {
  int seq = nextControlSeq();
  return buildBaseMessage("JOIN_CONFIRM", DEVICE_ID, repeaterId, seq);
}

String buildDataMessage(uint16_t seq, bool flood, const String& payload) {
  String msg = buildBaseMessage(flood ? "FLOOD" : "DATA", DEVICE_ID, repeaterId, seq);
  msg += ";TTL=";
  msg += flood ? (int)MAX_TTL : 1;
  msg += ";DATA=";
  msg += payload;
  return msg;
}

// ---------- Message Parser ----------

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
    token.trim();

    if (token.length() > 0) {
      int eq = token.indexOf('=');
      if (eq >= 0) {
        String key = token.substring(0, eq);
        String val = token.substring(eq + 1);
        key.trim();
        val.trim();

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
    }

    start = end + 1;
  }

  if (msg.type.length() > 0 && msg.hasSrc && msg.hasDst) {
    msg.valid = true;
  }

  return msg;
}

// ---------- Radio TX Helpers ----------

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

void sendAnnounce() {
  if (repeaterId < 0) return;

  String msg = buildAnnounceMessage();
  sendMessage(msg);

  lastAnnounceAt = millis();
  state = WAIT_REG_PING;

  Serial.print(F("[JOIN] Announce -> repeater "));
  Serial.println(repeaterId);
}

void sendJoinConfirm() {
  String msg = buildJoinConfirmMessage();
  sendMessage(msg);

  state = JOINED;
  uplink.waiting = false;

  Serial.println(F("[JOIN] Completed, node JOINED"));
}

// ---------- Uplink Handling ----------

void startUplink(bool floodMode) {
  uplink.waiting = true;
  uplink.floodMode = floodMode;
  uplink.seq = nextUplinkSeq();
  uplink.retries = 0;
  uplink.payload = getSensorReadings();
  uplink.lastAttempt = 0;
}

void transmitUplink() {
  if (!uplink.waiting) return;

  String msg = buildDataMessage(uplink.seq, uplink.floodMode, uplink.payload);
  sendMessage(msg);

  uplink.lastAttempt = millis();
  uplink.retries++;
}

void handleAck(const MeshMessage& msg) {
  if (!uplink.waiting || !msg.hasAck) return;

  if (msg.ack == uplink.seq && msg.src == repeaterId) {
    Serial.print(F("[ACK] Uplink confirmed seq="));
    Serial.println(uplink.seq);

    uplink.waiting = false;
    uplink.floodMode = false;
  }
}

// ---------- Beacon / Control Handling ----------

void handleBeacon(const MeshMessage& msg) {
  if (!msg.hasRank || msg.rank != 0) return;

  repeaterId = msg.src;
  lastBeaconHeard = millis();

  Serial.print(F("[DISCOVERY] Beacon from repeater "));
  Serial.println(repeaterId);

  if (state == WAIT_BEACON) {
    sendAnnounce();
  }
}

// ---------- RX Processing ----------

void processIncoming(const MeshMessage& msg) {
  if (!msg.valid) return;

  // Filter by destination: direct or broadcast (-1)
  if (msg.dst != DEVICE_ID && msg.dst != -1) return;

  if (msg.type == "BEACON") {
    handleBeacon(msg);

  } else if (msg.type == "REG_PING" && msg.src == repeaterId && state == WAIT_REG_PING) {

    Serial.println(F("[JOIN] REG_PING received"));
    sendJoinConfirm();

  } else if (msg.type == "ACK") {
    handleAck(msg);
  }
}

void pollRadio() {
  // Small burst of polling to keep latency down
  for (int i = 0; i < 3; i++) {
    String incoming;
    int16_t rxState = radio.receive(incoming);

    if (rxState == RADIOLIB_ERR_NONE) {
      MeshMessage msg = parseMessage(incoming);
      if (msg.valid) {
        Serial.print(F("[RX] "));
        Serial.println(incoming);
      }
      processIncoming(msg);

    } else if (rxState == RADIOLIB_ERR_RX_TIMEOUT || rxState == RADIOLIB_ERR_CRC_MISMATCH) {
      // no packet / bad packet -> stop this burst
      break;

    } else {
      // other radio error
      Serial.print(F("[RX][ERR] "));
      Serial.println(rxState);
      break;
    }
  }
}

// ======================= SETUP & LOOP =======================

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println();
  Serial.println(F("=== Rank-1 Sensor Mesh Node ==="));

  // I2C
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  // Sensors
  Serial.println(F("[INIT] Sensors..."));
  if (!aht20.begin()) {
    Serial.println(F("[INIT][WARN] AHT20 init failed"));
  }

  sensors.begin();

  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println(F("[INIT][WARN] BH1750 init failed"));
  }

  // LoRa radio
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
    USE_LDO);

  Serial.print(F("[INIT] radio.begin -> "));
  Serial.println(stateInit);

  if (stateInit != RADIOLIB_ERR_NONE) {
    Serial.println(F("[FATAL] LoRa init failed, check wiring/frequency/core"));
    while (true) {
      delay(1000);
    }
  }

  radio.setCRC(true);
  Serial.println(F("[INIT] Complete, waiting for BEACON"));
}

void loop() {
  pollRadio();

  unsigned long now = millis();

  switch (state) {
    case WAIT_BEACON:
      if (now - lastScanLog >= SCAN_INTERVAL_MS) {
        Serial.println(F("[DISCOVERY] Scanning for beacons..."));
        lastScanLog = now;
      }
      break;

    case WAIT_REG_PING:
      if (now - lastAnnounceAt > REG_PING_TIMEOUT_MS) {
        Serial.println(F("[JOIN] REG_PING timeout, back to discovery"));
        state = WAIT_BEACON;
        repeaterId = -1;
      }
      break;

    case JOINED:
      // Periodic uplink
      if (!uplink.waiting && (now - lastSendMillis >= SEND_INTERVAL_MS)) {
        startUplink(false);
        transmitUplink();
        lastSendMillis = now;
      }

      // Handle retries / flood failover
      if (uplink.waiting && (now - uplink.lastAttempt >= ACK_TIMEOUT_MS)) {
        if (!uplink.floodMode && uplink.retries < MAX_UPLINK_RETRIES) {
          Serial.println(F("[RETRY] No ACK, resending DATA"));
          transmitUplink();
        } else if (!uplink.floodMode) {
          Serial.println(F("[FAILOVER] Escalating to FLOOD mode"));
          uplink.floodMode = true;
          uplink.retries = 0;
          transmitUplink();
        } else if (uplink.retries < MAX_FLOOD_RETRIES) {
          Serial.println(F("[FLOOD] Resending FLOOD packet"));
          transmitUplink();
        } else {
          Serial.println(F("[ERROR] FLOOD retries exhausted, restart discovery"));
          uplink.waiting = false;
          uplink.floodMode = false;
          state = WAIT_BEACON;
          repeaterId = -1;
        }
      }

      // Beacon loss detection
      if (repeaterId >= 0 && (now - lastBeaconHeard > BEACON_LOSS_TIMEOUT_MS)) {
        Serial.println(F("[KEEPALIVE] Beacon lost, restarting discovery"));
        state = WAIT_BEACON;
        repeaterId = -1;
        uplink.waiting = false;
        uplink.floodMode = false;
      }
      break;
  }

  delay(50);
}