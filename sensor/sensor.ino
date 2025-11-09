#include <RadioLib.h>
#include <Wire.h>
#include <AHT20.h>
#include <DallasTemperature.h>
#include <BH1750.h>
#include <OneWire.h>

// ------------------- User Pin Configuration -------------------
#define I2C_SDA_PIN PB7
#define I2C_SCL_PIN PB6
#define ONE_WIRE_BUS PB0
#define SOIL_MOISTURE_PIN PA1

// ------------------- LoRa Parameters (FASTEST SETUP) -------------------
static const float   FREQ_MHZ  = 923.2;  // Japan band example
static const float   BW_KHZ    = 500.0;  // Maximum bandwidth
static const uint8_t SF        = 5;      // Lower SF = faster
static const uint8_t CR        = 5;      // 4/5
static const uint8_t SYNCWORD  = 0x12;   // Public LoRa
static const int8_t  PWR_DBM   = 22;     // Max TX power
static const uint16_t PREAMBLE = 8;
static const float    TCXO_V   = 1.6;
static const bool     USE_LDO  = false;

const int DEVICE_ID = 5;                // sensor rank-1 node
const uint8_t MAX_TTL = 3;              // maximum hops for flood recovery
const unsigned long SEND_INTERVAL_MS = 30000;
const unsigned long SCAN_INTERVAL_MS = 5000;
const unsigned long REG_PING_TIMEOUT_MS = 2000;
const unsigned long ACK_TIMEOUT_MS = 3000;
const uint8_t MAX_UPLINK_RETRIES = 3;
const uint8_t MAX_FLOOD_RETRIES = 2;

uint16_t uplinkSequence = 0;
uint16_t controlSequence = 0;

STM32WLx_Module wl;
STM32WLx radio(&wl);

// RF switch table (PA4 / PA5) - mandatory for Wio-E5 Mini
static const uint32_t RFSW_PINS[] = { PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC };
static const Module::RfSwitchMode_t RFSW_TABLE[] = {
  { STM32WLx::MODE_IDLE,  { LOW,  LOW  } },
  { STM32WLx::MODE_RX,    { HIGH, LOW  } },
  { STM32WLx::MODE_TX_LP, { LOW,  HIGH } },
  { STM32WLx::MODE_TX_HP, { LOW,  HIGH } },
  END_OF_MODE_TABLE,
};

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
AHT20 aht20;
BH1750 lightMeter;

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

SensorState state = WAIT_BEACON;
int repeaterId = -1;
unsigned long lastBeaconHeard = 0;
unsigned long lastAnnounceAt = 0;
unsigned long lastScanLog = 0;
unsigned long lastSendMillis = 0;

PendingUplink uplink;

String getSensorReadings() {
  sensors.requestTemperatures();
  float soilTemp = sensors.getTempCByIndex(0);
  int i = 0;
  while (soilTemp == 85.0f && i < 100) {
    delay(1);
    soilTemp = sensors.getTempCByIndex(0);
    i++;
  }

  float airTemp = aht20.getTemperature();
  float humidity = aht20.getHumidity();
  float lux = lightMeter.readLightLevel();
  int rawMoisture = analogRead(SOIL_MOISTURE_PIN);
  int moisturePercent = map(rawMoisture, 0, 4095, 0, 100);

  char msg[128];
  snprintf(msg, sizeof(msg),
           "ID:%d,ST:%.2f,AT:%.2f,H:%.2f,L:%.2f,M:%d",
           DEVICE_ID, soilTemp, airTemp, humidity, lux, moisturePercent);
  return String(msg);
}

uint16_t nextUplinkSeq() {
  return ++uplinkSequence;
}

uint16_t nextControlSeq() {
  return ++controlSequence;
}

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
  msg += (int)(flood ? MAX_TTL : 1);
  msg += ";DATA=";
  msg += payload;
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
    token.trim();
    if (token.length() > 0) {
      int eq = token.indexOf('=');
      if (eq >= 0) {
        String key = token.substring(0, eq);
        String value = token.substring(eq + 1);
        key.trim();
        value.trim();
        if (key == "TYPE") {
          msg.type = value;
        } else if (key == "SRC") {
          msg.src = value.toInt();
          msg.hasSrc = true;
        } else if (key == "DST") {
          msg.dst = value.toInt();
          msg.hasDst = true;
        } else if (key == "SEQ") {
          msg.seq = value.toInt();
          msg.hasSeq = true;
        } else if (key == "TTL") {
          msg.ttl = value.toInt();
          msg.hasTtl = true;
        } else if (key == "RANK") {
          msg.rank = value.toInt();
          msg.hasRank = true;
        } else if (key == "ACK") {
          msg.ack = value.toInt();
          msg.hasAck = true;
        } else if (key == "ORIG") {
          msg.orig = value.toInt();
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

void sendMessage(const String& payload) {
  int16_t state = radio.transmit(payload);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print(F("[TX] Sent: "));
    Serial.println(payload);
  } else {
    Serial.print(F("[TX] Failed code "));
    Serial.println(state);
  }
}

void sendAnnounce() {
  if (repeaterId < 0) {
    return;
  }
  String msg = buildAnnounceMessage();
  sendMessage(msg);
  lastAnnounceAt = millis();
  state = WAIT_REG_PING;
  Serial.print(F("[JOIN] Announce to repeater "));
  Serial.println(repeaterId);
}

void sendJoinConfirm() {
  String msg = buildJoinConfirmMessage();
  sendMessage(msg);
  state = JOINED;
  uplink.waiting = false;
  Serial.println(F("[JOIN] Network initialized"));
}

void startUplink(bool floodMode) {
  uplink.waiting = true;
  uplink.floodMode = floodMode;
  uplink.seq = nextUplinkSeq();
  uplink.retries = 0;
  uplink.payload = getSensorReadings();
  uplink.lastAttempt = 0;
}

void transmitUplink() {
  if (!uplink.waiting) {
    return;
  }
  String msg = buildDataMessage(uplink.seq, uplink.floodMode, uplink.payload);
  sendMessage(msg);
  uplink.lastAttempt = millis();
  uplink.retries++;
}

void handleAck(const MeshMessage& msg) {
  if (!uplink.waiting || !msg.hasAck) {
    return;
  }
  if (msg.ack == uplink.seq && msg.src == repeaterId) {
    Serial.print(F("[ACK] Hop confirmed for seq "));
    Serial.println(uplink.seq);
    uplink.waiting = false;
    uplink.floodMode = false;
  }
}

void handleBeacon(const MeshMessage& msg) {
  if (!msg.hasRank || msg.rank != 0) {
    return;
  }
  repeaterId = msg.src;
  lastBeaconHeard = millis();
  Serial.print(F("[DISCOVERY] Beacon from repeater "));
  Serial.println(repeaterId);
  if (state == WAIT_BEACON) {
    sendAnnounce();
  }
}

void processIncoming(const MeshMessage& msg) {
  if (!msg.valid) {
    return;
  }
  if (msg.dst != DEVICE_ID && msg.dst != -1) {
    return;
  }

  if (msg.type == "BEACON") {
    handleBeacon(msg);
  } else if (msg.type == "REG_PING" && msg.src == repeaterId && state == WAIT_REG_PING) {
    Serial.println(F("[JOIN] Registration ping received"));
    sendJoinConfirm();
  } else if (msg.type == "ACK") {
    handleAck(msg);
  }
}

void pollRadio() {
  for (int i = 0; i < 3; i++) {
    String incoming;
    int16_t stateRx = radio.receive(incoming);
    if (stateRx == RADIOLIB_ERR_NONE) {
      MeshMessage msg = parseMessage(incoming);
      if (msg.valid) {
        Serial.print(F("[RX] "));
        Serial.println(incoming);
      }
      processIncoming(msg);
    } else if (stateRx == RADIOLIB_ERR_RX_TIMEOUT || stateRx == RADIOLIB_ERR_CRC_MISMATCH) {
      break;
    } else {
      Serial.print(F("[RX] Error "));
      Serial.println(stateRx);
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println();
  Serial.println(F("=== Rank-1 Sensor Mesh Node ==="));

  // I2C setup
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  // Sensors
  Serial.println(F("Initializing sensors..."));
  if (!aht20.begin()) Serial.println(F("AHT20 init failed."));
  sensors.begin();
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
    Serial.println(F("BH1750 init failed."));

  // LoRa radio
  radio.setRfSwitchTable(RFSW_PINS, RFSW_TABLE);
  Serial.println(F("Applying RF switch table..."));

  int16_t stateInit = radio.begin(FREQ_MHZ, BW_KHZ, SF, CR, SYNCWORD, PWR_DBM, PREAMBLE, TCXO_V, USE_LDO);
  Serial.print(F("radio.begin -> ")); Serial.println(stateInit);
  if (stateInit != RADIOLIB_ERR_NONE) {
    Serial.println(F("LoRa init failed! Check wiring/core/frequency."));
    while (true) { delay(1000); }
  }

  radio.setCRC(true);
  Serial.println(F("Setup complete"));
}

void loop() {
  pollRadio();

  unsigned long now = millis();

  if (state == WAIT_BEACON) {
    if (now - lastScanLog >= SCAN_INTERVAL_MS) {
      Serial.println(F("[DISCOVERY] Scanning for beacons..."));
      lastScanLog = now;
    }
  } else if (state == WAIT_REG_PING) {
    if (now - lastAnnounceAt > REG_PING_TIMEOUT_MS) {
      Serial.println(F("[JOIN] Registration ping timeout, restarting discovery"));
      state = WAIT_BEACON;
    }
  } else if (state == JOINED) {
    if (!uplink.waiting && (now - lastSendMillis >= SEND_INTERVAL_MS)) {
      startUplink(false);
      transmitUplink();
      lastSendMillis = now;
    }

    if (uplink.waiting) {
      if (now - uplink.lastAttempt >= ACK_TIMEOUT_MS) {
        if (!uplink.floodMode && uplink.retries < MAX_UPLINK_RETRIES) {
          Serial.println(F("[RETRY] No ACK, resending data"));
          transmitUplink();
        } else if (!uplink.floodMode) {
          Serial.println(F("[FAILOVER] Escalating to flood"));
          uplink.floodMode = true;
          uplink.retries = 0;
          transmitUplink();
        } else if (uplink.retries < MAX_FLOOD_RETRIES) {
          Serial.println(F("[FLOOD] Re-sending flooded packet"));
          transmitUplink();
        } else {
          Serial.println(F("[ERROR] Flood retries exhausted, restarting discovery"));
          uplink.waiting = false;
          state = WAIT_BEACON;
        }
      }
    }

    if (now - lastBeaconHeard > 300000UL) {
      Serial.println(F("[KEEPALIVE] Beacon lost, restarting discovery"));
      state = WAIT_BEACON;
      repeaterId = -1;
      uplink.waiting = false;
    }
  }

  delay(50);
}
