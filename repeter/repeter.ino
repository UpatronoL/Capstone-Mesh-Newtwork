#include <RadioLib.h>
#include <string.h>

STM32WLx_Module wl;
STM32WLx radio(&wl);

static const uint32_t RFSW_PINS[] = { PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC };
static const Module::RfSwitchMode_t RFSW_TABLE[] = {
  { STM32WLx::MODE_IDLE,  { LOW,  LOW  } },
  { STM32WLx::MODE_RX,    { HIGH, LOW  } },
  { STM32WLx::MODE_TX_LP, { LOW,  HIGH } },
  { STM32WLx::MODE_TX_HP, { LOW,  HIGH } },
  END_OF_MODE_TABLE,
};

static const float   FREQ_MHZ  = 923.2;
static const float   BW_KHZ    = 500.0;
static const uint8_t SF        = 5;
static const uint8_t CR        = 5;
static const uint8_t SYNCWORD  = 0x12;
static const int8_t  PWR_DBM   = 14;
static const uint16_t PREAMBLE = 8;
static const float    TCXO_V   = 1.6;
static const bool     USE_LDO  = false;

static const int NODE_ID = 1;        // repeater identifier
static const int SINK_ID = 0;        // sink/receiver identifier

static const unsigned long BEACON_INTERVAL_MS = 5000;
static const unsigned long REGISTRATION_WINDOW_MS = 5UL * 60UL * 1000UL;
static const unsigned long FORWARD_ACK_TIMEOUT_MS = 3000;
static const uint8_t MAX_FORWARD_RETRIES = 3;

uint16_t beaconSeq = 0;
uint16_t controlSeq = 0;

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

struct ForwardState {
  bool active = false;
  int sensorId = -1;
  uint16_t seq = 0;
  int ttl = 0;
  String payload;
  bool flood = false;
  uint8_t retries = 0;
  unsigned long lastSend = 0;
};

bool sensorRegistered = false;
int registeredSensorId = -1;
unsigned long registrationExpiresAt = 0;
unsigned long lastBeaconSent = 0;

ForwardState forwardState;

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

uint16_t nextBeaconSeq() {
  return ++beaconSeq;
}

uint16_t nextControlSeq() {
  return ++controlSeq;
}

String buildBeaconMessage() {
  String msg = buildBaseMessage("BEACON", NODE_ID, -1, nextBeaconSeq());
  msg += ";RANK=0";
  return msg;
}

String buildRegistrationPing(int sensorId) {
  return buildBaseMessage("REG_PING", NODE_ID, sensorId, nextControlSeq());
}

String buildAckForSensor(uint16_t seq, int sensorId) {
  String msg = buildBaseMessage("ACK", NODE_ID, sensorId, nextControlSeq());
  msg += ";ACK=";
  msg += seq;
  return msg;
}

String buildForwardPacket(uint16_t seq, int ttl, int sensorId, const String& payload) {
  String msg = buildBaseMessage("DATA", NODE_ID, SINK_ID, seq);
  msg += ";TTL=";
  msg += ttl;
  msg += ";ORIG=";
  msg += sensorId;
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

void sendBeacon() {
  String msg = buildBeaconMessage();
  sendMessage(msg);
  lastBeaconSent = millis();
}

void handleAnnounce(const MeshMessage& msg) {
  registeredSensorId = msg.src;
  sensorRegistered = true;
  registrationExpiresAt = millis() + REGISTRATION_WINDOW_MS;
  Serial.print(F("[JOIN] Sensor announced: "));
  Serial.println(registeredSensorId);
  sendMessage(buildRegistrationPing(registeredSensorId));
}

void handleJoinConfirm(const MeshMessage& msg) {
  if (!sensorRegistered || msg.src != registeredSensorId) {
    return;
  }
  registrationExpiresAt = millis() + REGISTRATION_WINDOW_MS;
  Serial.println(F("[JOIN] Sensor confirmed join"));
}

void forwardToSink() {
  if (!forwardState.active) {
    return;
  }
  if (forwardState.ttl < 0) {
    Serial.println(F("[FWD] TTL expired, dropping packet"));
    forwardState.active = false;
    return;
  }
  String packet = buildForwardPacket(forwardState.seq, forwardState.ttl, forwardState.sensorId, forwardState.payload);
  sendMessage(packet);
  forwardState.lastSend = millis();
  forwardState.retries++;
}

void acknowledgeSensor() {
  if (!forwardState.active) {
    return;
  }
  String ack = buildAckForSensor(forwardState.seq, forwardState.sensorId);
  sendMessage(ack);
  forwardState.active = false;
}

void handleDataFromSensor(const MeshMessage& msg) {
  if (!msg.hasSeq || !msg.hasTtl || !msg.hasData) {
    Serial.println(F("[RX] Malformed data from sensor"));
    return;
  }

  if (sensorRegistered && msg.src != registeredSensorId) {
    Serial.println(F("[RX] Ignoring data from unregistered sensor"));
    return;
  }

  if (msg.ttl <= 0) {
    Serial.println(F("[RX] Dropping packet with expired TTL"));
    return;
  }

  if (forwardState.active) {
    if (msg.seq == forwardState.seq && msg.src == forwardState.sensorId) {
      Serial.println(F("[FWD] Sensor retransmission detected, retrying forward"));
      forwardState.ttl = msg.ttl - 1;
      forwardState.payload = msg.data;
      forwardState.flood = (msg.type == "FLOOD");
      forwardState.retries = 0;
      forwardToSink();
    } else {
      Serial.println(F("[FWD] Busy handling previous packet, ignoring new uplink"));
    }
    return;
  }

  forwardState.active = true;
  forwardState.sensorId = msg.src;
  forwardState.seq = msg.seq;
  forwardState.ttl = msg.ttl - 1;
  forwardState.payload = msg.data;
  forwardState.flood = (msg.type == "FLOOD");
  forwardState.retries = 0;

  Serial.print(F("[FWD] Forwarding seq "));
  Serial.print(forwardState.seq);
  Serial.print(F(" flood="));
  Serial.println(forwardState.flood ? F("yes") : F("no"));

  forwardToSink();
}

void handleAckFromSink(const MeshMessage& msg) {
  if (!msg.hasAck) {
    return;
  }
  if (!forwardState.active) {
    return;
  }
  if (msg.ack != forwardState.seq) {
    return;
  }

  Serial.print(F("[ACK] Sink confirmed seq "));
  Serial.println(msg.ack);
  acknowledgeSensor();
}

void processIncoming(const MeshMessage& msg) {
  if (!msg.valid) {
    return;
  }

  if (msg.type == "ANNOUNCE" && msg.dst == NODE_ID) {
    handleAnnounce(msg);
  } else if (msg.type == "JOIN_CONFIRM" && msg.dst == NODE_ID) {
    handleJoinConfirm(msg);
  } else if ((msg.type == "DATA" || msg.type == "FLOOD") && msg.dst == NODE_ID) {
    handleDataFromSensor(msg);
  } else if (msg.type == "ACK" && msg.dst == NODE_ID && msg.src == SINK_ID) {
    handleAckFromSink(msg);
  }
}

void pollRadio() {
  for (int i = 0; i < 4; i++) {
    String incoming;
    int16_t stateRx = radio.receive(incoming);
    if (stateRx == RADIOLIB_ERR_NONE) {
      Serial.print(F("[RX] "));
      Serial.println(incoming);
      MeshMessage msg = parseMessage(incoming);
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

  radio.setRfSwitchTable(RFSW_PINS, RFSW_TABLE);
  Serial.println(F("Applying RF switch table"));

  int16_t stateInit = radio.begin(FREQ_MHZ, BW_KHZ, SF, CR, SYNCWORD, PWR_DBM, PREAMBLE, TCXO_V, USE_LDO);
  Serial.print(F("radio.begin -> "));
  Serial.println(stateInit);
  if (stateInit != RADIOLIB_ERR_NONE) {
    Serial.println(F("LoRa init failed! Check wiring/core/frequency."));
    while (true) { delay(1000); }
  }

  radio.setCRC(true);
  Serial.println(F("Repeater ready"));
}

void loop() {
  unsigned long now = millis();

  if (now - lastBeaconSent >= BEACON_INTERVAL_MS) {
    sendBeacon();
  }

  pollRadio();

  if (sensorRegistered && now > registrationExpiresAt) {
    Serial.println(F("[JOIN] Registration expired"));
    sensorRegistered = false;
    registeredSensorId = -1;
  }

  if (forwardState.active && (now - forwardState.lastSend) > FORWARD_ACK_TIMEOUT_MS) {
    if (forwardState.retries < MAX_FORWARD_RETRIES) {
      Serial.println(F("[FWD] No ACK from sink, retrying"));
      forwardToSink();
    } else {
      Serial.println(F("[FWD] Exhausted retries waiting for sink ACK"));
      forwardState.active = false;
    }
  }

  delay(50);
}
