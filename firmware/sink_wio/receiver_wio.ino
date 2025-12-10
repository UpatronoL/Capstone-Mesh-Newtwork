#include <Arduino.h>
#include <RadioLib.h>
#include <string>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <stddef.h>   // for offsetof

// ================== Radio / Mesh Config ==================

// Wio-E5 uses STM32WLx core inside the module
STM32WLx_Module wl;
STM32WLx radio(&wl);

static const uint8_t SINK_ID           = 0;     // This node (base)
static const uint8_t DST_BROADCAST     = 255;   // optional broadcast ID

// Keep LoRa settings aligned with repeater/sensor
static const float   LORA_FREQ_MHZ   = 923.2f;
static const float   LORA_BW_KHZ     = 500.0f;
static const uint8_t LORA_SF         = 5;
static const uint8_t LORA_CR         = 5;
static const uint8_t LORA_SYNCWORD   = 0x12;
static const int8_t  LORA_PWR_DBM    = -8;
static const uint16_t LORA_PREAMBLE  = 8;
static const float   LORA_TCXO_V     = 1.6f;
static const bool    LORA_USE_LDO    = false;

// Timing / dedup
static const unsigned long BEACON_INTERVAL_MS = 15000UL;
static const unsigned long BEACON_JITTER_MS   = 3000UL;
static const unsigned long DEDUP_TIMEOUT_MS   = 30000UL;

// Must match network config (repeater & sensor)
static const uint8_t MAX_TTL = 10;

// ================== Packet types (must match repeater) ==================

enum PacketType : uint8_t {
  PKT_BEACON   = 0,
  PKT_DATA     = 1,
  PKT_ACK      = 2,
  PKT_JOIN_REQ = 3,
  PKT_JOIN_ACK = 4
};

static uint16_t beaconSeq = 0;

// ================== Binary packet formats (must match repeater) ==================

static const size_t MAX_PAYLOAD_LEN = 64;
static const size_t RX_BUFFER_SIZE  = 128;

struct __attribute__((packed)) SensorPayload {
  uint8_t  nodeId;      // 1 byte
  uint32_t counter;     // 4 bytes
  int16_t  tempAir;     // 2 bytes (x100)
  int16_t  tempSoil;    // 2 bytes (x100)
  uint16_t humidity;    // 2 bytes (x100)
  uint16_t lux;         // 2 bytes
  uint8_t  moisture;    // 1 byte
};

struct __attribute__((packed)) BeaconPacket {
  uint8_t  type;   // PKT_BEACON
  uint8_t  srcId;
  uint8_t  rank;
  uint16_t seq;
  uint8_t  crc;
};

struct __attribute__((packed)) DataPacket {
  uint8_t  type;     // PKT_DATA
  uint8_t  srcId;    // immediate sender
  uint8_t  dstId;    // next hop (often SINK_ID)
  uint8_t  ttl;
  uint8_t  origSrc;  // original sensor ID
  uint16_t origSeq;  // original sensor sequence
  uint16_t seq;      // repeater's own seq
  uint8_t  payloadLen;
  uint8_t  payload[MAX_PAYLOAD_LEN];  // variable portion
  uint8_t  crc;
};

struct __attribute__((packed)) AckPacket {
  uint8_t  type;   // PKT_ACK
  uint8_t  srcId;
  uint8_t  dstId;
  uint16_t ackSeq;  // original sensor seq (origSeq)
  uint8_t  crc;
};

struct __attribute__((packed)) JoinReqPacket {
  uint8_t  type;   // PKT_JOIN_REQ
  uint8_t  srcId;
  uint8_t  parentId;
  uint16_t seq;
  uint8_t  crc;
};

struct __attribute__((packed)) JoinAckPacket {
  uint8_t  type;   // PKT_JOIN_ACK
  uint8_t  srcId;
  uint8_t  childId;
  uint8_t  crc;
};

// ================== Deduplication Cache =====================

struct SeenEntry {
  int           origId = -1;
  int           seq    = -1;
  unsigned long ts     = 0;
  bool          used   = false;
};

static const size_t SEEN_CACHE_SIZE = 32;
SeenEntry seenCache[SEEN_CACHE_SIZE];

// ================== Small helpers =====================

// Simple wrap-safe time check
static bool isWithin(unsigned long now, unsigned long start, unsigned long interval) {
  return (now - start) < interval;
}

// Simple CRC-8 (poly 0x07, init 0x00) – must match repeater
static uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// ================== Timestamp + CSV logging =====================

static std::string getCurrentTimestamp() {
  char buf[16];
  unsigned long ms = millis();
  snprintf(buf, sizeof(buf), "%lu", ms);
  return std::string(buf);
}

// CSV output over Serial (one-time header)
static bool csvHeaderPrinted = false;

static void logBinarySensorData(const SensorPayload& sp,
                                int   origId,
                                int   seq,
                                float rssi,
                                float snr,
                                int   hops) {

  if (!csvHeaderPrinted) {
    Serial.println(F("TimestampMs,NodeID,Counter,SoilTemp_C,AirTemp_C,Humidity_%,Light_Lux,Moisture_%,RSSI_dBm,SNR_dB,Sequence,Hops"));
    csvHeaderPrinted = true;
  }

  std::string timestamp = getCurrentTimestamp();

  // Convert fixed-point integers back to floats
  float airTemp  = sp.tempAir / 100.0f;
  float soilTemp = sp.tempSoil / 100.0f;
  float hum      = sp.humidity / 100.0f;

  Serial.print(timestamp.c_str()); Serial.print(',');
  Serial.print(sp.nodeId);        Serial.print(',');
  Serial.print(sp.counter);       Serial.print(',');
  Serial.print(soilTemp, 2);      Serial.print(',');
  Serial.print(airTemp, 2);       Serial.print(',');
  Serial.print(hum, 2);           Serial.print(',');
  Serial.print(sp.lux);           Serial.print(',');
  Serial.print(sp.moisture);      Serial.print(',');
  Serial.print(rssi, 1);          Serial.print(',');
  Serial.print(snr, 1);           Serial.print(',');
  Serial.print(seq);              Serial.print(',');
  Serial.println(hops);

  Serial.print(F("[CSV] Logged binary data from node "));
  Serial.println(sp.nodeId);
}

// ================== Dedup helpers =====================

static void pruneSeenCache() {
  unsigned long now = millis();
  for (size_t i = 0; i < SEEN_CACHE_SIZE; ++i) {
    if (seenCache[i].used && (now - seenCache[i].ts > DEDUP_TIMEOUT_MS)) {
      seenCache[i].used = false;
    }
  }
}

static bool hasSeenPacket(int origId, int seqNum) {
  for (size_t i = 0; i < SEEN_CACHE_SIZE; ++i) {
    if (seenCache[i].used &&
        seenCache[i].origId == origId &&
        seenCache[i].seq    == seqNum) {
      return true;
    }
  }
  return false;
}

static void addToSeenCache(int origId, int seqNum) {
  unsigned long now = millis();

  int freeIdx = -1;
  unsigned long oldestTs = 0xFFFFFFFF;
  int oldestIdx = 0;

  for (size_t i = 0; i < SEEN_CACHE_SIZE; ++i) {
    if (!seenCache[i].used) {
      freeIdx = (int)i;
      break;
    }
    if (seenCache[i].ts < oldestTs) {
      oldestTs = seenCache[i].ts;
      oldestIdx = (int)i;
    }
  }

  int idx = (freeIdx >= 0) ? freeIdx : oldestIdx;
  seenCache[idx].used   = true;
  seenCache[idx].origId = origId;
  seenCache[idx].seq    = seqNum;
  seenCache[idx].ts     = now;
}

// ================== ACK & BEACON (binary, repeater-compatible) =====================

static void transmitAck(uint16_t origSeq, uint8_t senderId) {
  AckPacket pkt;
  pkt.type   = PKT_ACK;
  pkt.srcId  = SINK_ID;
  pkt.dstId  = senderId;
  pkt.ackSeq = origSeq;  // important: original sensor sequence
  pkt.crc    = crc8((uint8_t*)&pkt, sizeof(pkt) - 1);

  int16_t state = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print(F("[ACK] Sent to "));
    Serial.print(senderId);
    Serial.print(F(" for origSeq="));
    Serial.println(origSeq);
  } else {
    Serial.print(F("[ACK] Failed (code="));
    Serial.print(state);
    Serial.println(')');
  }
}

static void transmitBeacon() {
  // Binary beacon: repeater expects srcId=0, rank=0
  long jitter = random(BEACON_JITTER_MS);
  if (jitter > 0) {
    delay(jitter);  // small random spacing; OK since sink just listens otherwise
  }

  BeaconPacket pkt;
  pkt.type  = PKT_BEACON;
  pkt.srcId = SINK_ID;
  pkt.rank  = 0;              // sink rank 0
  pkt.seq   = ++beaconSeq;
  pkt.crc   = crc8((uint8_t*)&pkt, sizeof(pkt) - 1);

  int16_t state = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print(F("[BEACON] Sent rank=0 seq="));
    Serial.println(beaconSeq);
  } else {
    Serial.print(F("[BEACON] Failed (code="));
    Serial.print(state);
    Serial.println(')');
  }
}

// ================== Packet handling (binary) =====================

// Handle BEACON: for now sink only logs
static void handleBeaconPacket(const BeaconPacket& pkt, float rssi, float snr) {
  Serial.print(F("[RX] BEACON from "));
  Serial.print(pkt.srcId);
  Serial.print(F(" rank="));
  Serial.print(pkt.rank);
  Serial.print(F(" seq="));
  Serial.print(pkt.seq);
  Serial.print(F(" rssi="));
  Serial.print(rssi, 1);
  Serial.print(F(" dBm snr="));
  Serial.print(snr, 1);
  Serial.println(F(" dB"));
}

// Handle DATA: parse payload ASCII & log, send ACK
static void handleDataPacket(const DataPacket& pkt, float rssi, float snr) {
  // Dedup on (origSrc, origSeq)
  int originatorId = pkt.origSrc;
  int originSeq    = pkt.origSeq;

  if (hasSeenPacket(originatorId, originSeq)) {
    Serial.print(F("[DEDUP] Already saw orig="));
    Serial.print(originatorId);
    Serial.print(F(" seq="));
    Serial.println(originSeq);

    // Still ACK so repeater stops retrying
    transmitAck(originSeq, pkt.srcId);
    return;
  }
  addToSeenCache(originatorId, originSeq);

  // TTL -> hops
  int hops = 0;
  if (pkt.ttl <= MAX_TTL) {
    hops = (int)MAX_TTL - pkt.ttl;
    if (hops < 0) hops = 0;
  }

  // bin handling
  if (pkt.payloadLen != sizeof(SensorPayload)) {
    Serial.print(F("[RX] Payload length mismatch. Expected "));
    Serial.print(sizeof(SensorPayload));
    Serial.print(F(" but got "));
    Serial.println(pkt.payloadLen);
    return; 
  }

  SensorPayload sp;
  // Copy raw bytes from packet payload into our struct
  memcpy(&sp, pkt.payload, sizeof(SensorPayload));

  Serial.print(F("[RX] BINARY DATA os="));
  Serial.print(originatorId);
  Serial.print(F(" oq="));
  Serial.print(originSeq);
  Serial.print(F(" hops="));
  Serial.println(hops);

  // Pass struct to the new logging function
  logBinarySensorData(sp, originatorId, originSeq, rssi, snr, hops);

  // ACK back to immediate sender
  transmitAck(originSeq, pkt.srcId);
}

// JOIN_REQ / JOIN_ACK currently not used by sink, but we at least log them
static void handleJoinReqPacket(const JoinReqPacket& pkt) {
  Serial.print(F("[RX] JOIN_REQ from "));
  Serial.print(pkt.srcId);
  Serial.print(F(" -> parent "));
  Serial.println(pkt.parentId);
}

static void handleJoinAckPacket(const JoinAckPacket& pkt) {
  Serial.print(F("[RX] JOIN_ACK child="));
  Serial.print(pkt.childId);
  Serial.print(F(" from "));
  Serial.println(pkt.srcId);
}

// ================== Arduino setup/loop =====================

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  Serial.println();
  Serial.println(F("=== STM32WLE5 Binary Mesh Sink Node (Rank 0) ==="));
  Serial.println(F("[INIT] Initializing radio..."));

  randomSeed(analogRead(A0));   // any noisy pin works as seed

  static const uint32_t RFSW_PINS[] = {
    PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
  };

  static const Module::RfSwitchMode_t RFSW_TABLE[] = {
    { STM32WLx::MODE_IDLE,  { LOW,  LOW  } },
    { STM32WLx::MODE_RX,    { HIGH, LOW  } },
    { STM32WLx::MODE_TX_LP, { LOW,  HIGH } },
    { STM32WLx::MODE_TX_HP, { LOW,  HIGH } },
    END_OF_MODE_TABLE,
  };

  radio.setRfSwitchTable(RFSW_PINS, RFSW_TABLE);
  Serial.println(F("[INIT] RF switch table applied"));

  int16_t state = radio.begin(
    LORA_FREQ_MHZ,
    LORA_BW_KHZ,
    LORA_SF,
    LORA_CR,
    LORA_SYNCWORD,
    LORA_PWR_DBM,
    LORA_PREAMBLE,
    LORA_TCXO_V,
    LORA_USE_LDO
  );

  Serial.print(F("[INIT] radio.begin -> "));
  Serial.println(state);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("[INIT] Radio init failed (code="));
    Serial.print(state);
    Serial.println(')');
    while (true) {
      delay(1000);
    }
  }

  radio.setCRC(true);  // PHY-level CRC

  Serial.println(F("[INIT] Radio configured:"));
  Serial.print(F("       Freq: "));   Serial.print(LORA_FREQ_MHZ); Serial.println(F(" MHz"));
  Serial.print(F("       BW: "));     Serial.print(LORA_BW_KHZ);   Serial.println(F(" kHz"));
  Serial.print(F("       SF: "));     Serial.println(LORA_SF);
  Serial.print(F("       CR: 4/"));   Serial.println(LORA_CR);
  Serial.print(F("       Power: "));  Serial.print(LORA_PWR_DBM);  Serial.println(F(" dBm"));
  Serial.println(F("[INIT] Ready. Listening for packets...\n"));
}

void loop() {
  static unsigned long lastBeacon     = 0;
  static unsigned long lastCachePrune = 0;

  unsigned long now = millis();

  // Periodic beacon transmission
  if (!isWithin(now, lastBeacon, BEACON_INTERVAL_MS)) {
    transmitBeacon();
    lastBeacon = millis();
  }

  // Periodic dedup cache pruning
  if (!isWithin(now, lastCachePrune, 10000UL)) {
    pruneSeenCache();
    lastCachePrune = millis();
  }

  // Binary receive
  uint8_t rxBuf[RX_BUFFER_SIZE];
  int16_t state = radio.receive(rxBuf, RX_BUFFER_SIZE);

  if (state == RADIOLIB_ERR_NONE) {
    size_t len = radio.getPacketLength();
    if (len == 0 || len > RX_BUFFER_SIZE) {
      Serial.println(F("[RX] Malformed length"));
      return;
    }

    float rssi = radio.getRSSI();
    float snr  = radio.getSNR();

    uint8_t type = rxBuf[0];

    switch (type) {
      case PKT_BEACON: {
        if (len < sizeof(BeaconPacket)) {
          Serial.println(F("[RX] Beacon too short"));
          break;
        }
        BeaconPacket bp;
        memcpy(&bp, rxBuf, sizeof(BeaconPacket));
        uint8_t calc = crc8((uint8_t*)&bp, sizeof(bp) - 1);
        if (calc != bp.crc) {
          Serial.println(F("[CRC] Beacon CRC error"));
          break;
        }
        handleBeaconPacket(bp, rssi, snr);
        break;
      }

      case PKT_DATA: {
        size_t headerSize = offsetof(DataPacket, payload);
        if (len < headerSize + 1) {
          Serial.println(F("[RX] Data too short (header)"));
          break;
        }

        DataPacket dp;
        memset(&dp, 0, sizeof(dp));
        memcpy(&dp, rxBuf, headerSize);  // header + payloadLen

        if (dp.payloadLen > MAX_PAYLOAD_LEN) {
          Serial.println(F("[RX] Data payload too large"));
          break;
        }

        size_t needed = headerSize + dp.payloadLen + 1; // + crc
        if (len < needed) {
          Serial.println(F("[RX] Data length mismatch"));
          break;
        }

        memcpy(dp.payload, rxBuf + headerSize, dp.payloadLen);
        dp.crc = rxBuf[headerSize + dp.payloadLen];

        uint8_t calc = crc8((uint8_t*)&dp, headerSize + dp.payloadLen);
        if (calc != dp.crc) {
          Serial.println(F("[CRC] Data CRC error"));
          break;
        }

        handleDataPacket(dp, rssi, snr);
        break;
      }

      case PKT_ACK: {
        if (len < sizeof(AckPacket)) {
          Serial.println(F("[RX] Ack too short"));
          break;
        }
        AckPacket ap;
        memcpy(&ap, rxBuf, sizeof(AckPacket));
        uint8_t calc = crc8((uint8_t*)&ap, sizeof(ap) - 1);
        if (calc != ap.crc) {
          Serial.println(F("[CRC] Ack CRC error"));
          break;
        }
        handleJoinAckPacket(*(JoinAckPacket*)&ap); // or just log as "ACK"; sink doesn't need it
        Serial.println(F("[RX] ACK (sink currently ignores for logic)"));
        break;
      }

      case PKT_JOIN_REQ: {
        if (len < sizeof(JoinReqPacket)) {
          Serial.println(F("[RX] JoinReq too short"));
          break;
        }
        JoinReqPacket jp;
        memcpy(&jp, rxBuf, sizeof(JoinReqPacket));
        uint8_t calc = crc8((uint8_t*)&jp, sizeof(jp) - 1);
        if (calc != jp.crc) {
          Serial.println(F("[CRC] JoinReq CRC error"));
          break;
        }
        handleJoinReqPacket(jp);
        break;
      }

      case PKT_JOIN_ACK: {
        if (len < sizeof(JoinAckPacket)) {
          Serial.println(F("[RX] JoinAck too short"));
          break;
        }
        JoinAckPacket ja;
        memcpy(&ja, rxBuf, sizeof(JoinAckPacket));
        uint8_t calc = crc8((uint8_t*)&ja, sizeof(ja) - 1);
        if (calc != ja.crc) {
          Serial.println(F("[CRC] JoinAck CRC error"));
          break;
        }
        handleJoinAckPacket(ja);
        break;
      }

      default:
        Serial.print(F("[RX] Unknown type "));
        Serial.println(type);
        break;
    }

  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // expected - nothing received during timeout
  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    Serial.println(F("[RX] PHY CRC mismatch"));
  } else {
    Serial.print(F("[RX] Receive failed (code="));
    Serial.print(state);
    Serial.println(')');
  }
}
