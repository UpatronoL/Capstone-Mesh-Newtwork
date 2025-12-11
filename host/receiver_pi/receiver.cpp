#include <RadioLib.h>
#include "hal/RPi/PiHal.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>

// ================== Radio / Mesh Config ==================

static PiHal* hal = new PiHal(0);                 // SPI channel 0 (CE0)
// NSS=8, DIO1=25, NRST=23, BUSY=24
static SX1262 radio = new Module(hal, 8, 25, 23, 24);

static const uint8_t SINK_ID       = 0;           // This node (base)
static const uint8_t DST_BROADCAST = 255;

// Keep LoRa settings aligned with repeater/sensor/Wio-E5 sink
static const float   LORA_FREQ_MHZ  = 923.2f;
static const float   LORA_BW_KHZ    = 500.0f;
static const uint8_t LORA_SF        = 5;
static const uint8_t LORA_CR        = 5;
static const uint8_t LORA_SYNCWORD  = 0x12;
// Use same TX power as Wio-E5 sink demo (can change if needed)
static const int8_t  LORA_PWR_DBM   = -8;
static const uint16_t LORA_PREAMBLE = 8;

// Timing / dedup
static const unsigned long BEACON_INTERVAL_MS = 15000UL;
static const unsigned long BEACON_JITTER_MS   = 3000UL;
static const unsigned long DEDUP_TIMEOUT_MS   = 30000UL;
static const unsigned long RX_TIMEOUT_MS      = 100UL;

// For hops calculation
static const uint8_t MAX_TTL = 10;

static uint16_t beaconSeq = 0;

// ================== "millis()" helper for Pi =====================

// Arduino-style millis() using steady_clock
static unsigned long millis() {
  using namespace std::chrono;
  static const auto start = steady_clock::now();
  return (unsigned long)duration_cast<milliseconds>(
             steady_clock::now() - start
         ).count();
}

// ================== Packet types (must match repeater) ==================

enum PacketType : uint8_t {
  PKT_BEACON   = 0,
  PKT_DATA     = 1,
  PKT_ACK      = 2,
  PKT_JOIN_REQ = 3,
  PKT_JOIN_ACK = 4
};

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
static SeenEntry seenCache[SEEN_CACHE_SIZE];

// ================== Small helpers =====================

// Simple wrap-safe time check
static bool isWithin(unsigned long now, unsigned long start, unsigned long interval) {
  return (now - start) < interval;
}

// Simple CRC-8 (poly 0x07, init 0x00) – must match repeater + Wio-E5 sink
static uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) {
        crc = (uint8_t)((crc << 1) ^ 0x07);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// ================== CSV logging to file =====================

static std::ofstream csvFile;
static const char* CSV_FILENAME = "sensor_data_binary.csv";
static bool csvHeaderPrinted = false;

static std::string getCurrentTimestamp() {
  using namespace std::chrono;
  auto now = system_clock::now();
  auto time_t_now = system_clock::to_time_t(now);
  auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

  std::tm tm_now;
  localtime_r(&time_t_now, &tm_now);

  std::ostringstream oss;
  oss << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
  oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
  return oss.str();
}

static void initCSV() {
  csvFile.open(CSV_FILENAME, std::ios::app);
  if (!csvFile.is_open()) {
    std::printf("[CSV] Failed to open %s\n", CSV_FILENAME);
    return;
  }

  // If file is empty, write header
  csvFile.seekp(0, std::ios::end);
  if (csvFile.tellp() == 0) {
    csvFile << "Timestamp,NodeID,Counter,SoilTemp_C,AirTemp_C,Humidity_%,"
               "Light_Lux,Moisture_%,RSSI_dBm,SNR_dB,Sequence,Hops\n";
    csvFile.flush();
    std::printf("[CSV] Created new binary CSV file with header: %s\n", CSV_FILENAME);
  } else {
    std::printf("[CSV] Appending to existing CSV file: %s\n", CSV_FILENAME);
  }

  csvHeaderPrinted = true;
}

static void logBinarySensorData(const SensorPayload& sp,
                                int   origId,
                                int   seq,
                                float rssi,
                                float snr,
                                int   hops) {

  if (!csvFile.is_open()) {
    std::printf("[CSV] File not open, skipping log\n");
    return;
  }

  if (!csvHeaderPrinted) {
    // Safety: header should already be there from initCSV, but just in case
    csvFile << "Timestamp,NodeID,Counter,SoilTemp_C,AirTemp_C,Humidity_%,"
               "Light_Lux,Moisture_%,RSSI_dBm,SNR_dB,Sequence,Hops\n";
    csvHeaderPrinted = true;
  }

  std::string timestamp = getCurrentTimestamp();

  // Convert fixed-point integers back to floats
  float airTemp  = sp.tempAir  / 100.0f;
  float soilTemp = sp.tempSoil / 100.0f;
  float hum      = sp.humidity / 100.0f;

  csvFile << timestamp << ','
          << (int)sp.nodeId << ','
          << sp.counter << ','
          << std::fixed << std::setprecision(2)
          << soilTemp << ','
          << airTemp  << ','
          << hum      << ','
          << sp.lux   << ','
          << (int)sp.moisture << ','
          << std::setprecision(1)
          << rssi << ','
          << snr  << ','
          << seq  << ','
          << hops << '\n';

  csvFile.flush();

  std::printf("[CSV] Logged binary data from node %d (seq=%d, hops=%d)\n",
              (int)sp.nodeId, seq, hops);
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
    std::printf("[ACK] Sent to %u for origSeq=%u\n",
                (unsigned)senderId, (unsigned)origSeq);
  } else {
    std::printf("[ACK] Failed (code=%d)\n", state);
  }
}

static void transmitBeacon() {
  // Binary beacon: repeater expects srcId=0, rank=0
  long jitter = std::rand() % (int)BEACON_JITTER_MS;
  if (jitter > 0) {
    hal->delay((unsigned long)jitter);  // small random spacing
  }

  BeaconPacket pkt;
  pkt.type  = PKT_BEACON;
  pkt.srcId = SINK_ID;
  pkt.rank  = 0;              // sink rank 0
  pkt.seq   = ++beaconSeq;
  pkt.crc   = crc8((uint8_t*)&pkt, sizeof(pkt) - 1);

  int16_t state = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
  if (state == RADIOLIB_ERR_NONE) {
    std::printf("[BEACON] Sent rank=0 seq=%u\n", (unsigned)beaconSeq);
  } else {
    std::printf("[BEACON] Failed (code=%d)\n", state);
  }
}

// ================== Packet handling (binary) =====================

// BEACON: for now sink only logs
static void handleBeaconPacket(const BeaconPacket& pkt, float rssi, float snr) {
  std::printf("[RX] BEACON from %u rank=%u seq=%u rssi=%.1f dBm snr=%.1f dB\n",
              (unsigned)pkt.srcId,
              (unsigned)pkt.rank,
              (unsigned)pkt.seq,
              rssi, snr);
}

// DATA: parse binary payload & log, send ACK
static void handleDataPacket(const DataPacket& pkt, float rssi, float snr) {
  int originatorId = pkt.origSrc;
  int originSeq    = pkt.origSeq;

  // Dedup on (origSrc, origSeq)
  if (hasSeenPacket(originatorId, originSeq)) {
    std::printf("[DEDUP] Already saw orig=%d seq=%d\n", originatorId, originSeq);
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

  // Payload length must match SensorPayload
  if (pkt.payloadLen != sizeof(SensorPayload)) {
    std::printf("[RX] Payload length mismatch. Expected %zu but got %u\n",
                sizeof(SensorPayload), (unsigned)pkt.payloadLen);
    // We can still ACK so repeater doesn't keep retrying
    transmitAck(originSeq, pkt.srcId);
    return;
  }

  SensorPayload sp;
  std::memcpy(&sp, pkt.payload, sizeof(SensorPayload));

  std::printf("[RX] BINARY DATA os=%d oq=%d hops=%d rssi=%.1f dBm snr=%.1f dB\n",
              originatorId, originSeq, hops, rssi, snr);

  // Log to CSV
  logBinarySensorData(sp, originatorId, originSeq, rssi, snr, hops);

  // ACK back to immediate sender
  transmitAck(originSeq, pkt.srcId);
}

// JOIN_REQ / JOIN_ACK currently not used for logic, but we log them
static void handleJoinReqPacket(const JoinReqPacket& pkt) {
  std::printf("[RX] JOIN_REQ from %u -> parent %u seq=%u\n",
              (unsigned)pkt.srcId,
              (unsigned)pkt.parentId,
              (unsigned)pkt.seq);
}

static void handleJoinAckPacket(const JoinAckPacket& pkt) {
  std::printf("[RX] JOIN_ACK child=%u from %u\n",
              (unsigned)pkt.childId,
              (unsigned)pkt.srcId);
}

// ================== Main =====================

int main() {
  std::printf("=== SX1262 Binary Mesh Sink Node (Rank 0) ===\n");
  std::printf("[INIT] Initializing receiver...\n");

  std::srand((unsigned)std::time(nullptr));

  // Init CSV
  initCSV();

  // Initialize radio with matching parameters
  int16_t state = radio.begin(
      LORA_FREQ_MHZ,
      LORA_BW_KHZ,
      LORA_SF,
      LORA_CR,
      LORA_SYNCWORD,
      LORA_PWR_DBM,
      LORA_PREAMBLE
  );

  if (state != RADIOLIB_ERR_NONE) {
    std::printf("[INIT] Radio init failed (code=%d)\n", state);
    return 1;
  }

  radio.setCRC(true);  // PHY-level CRC

  std::printf("[INIT] Radio configured:\n");
  std::printf("       Freq: %.2f MHz\n", LORA_FREQ_MHZ);
  std::printf("       BW: %.1f kHz\n", LORA_BW_KHZ);
  std::printf("       SF: %d\n", LORA_SF);
  std::printf("       CR: 4/%d\n", LORA_CR);
  std::printf("       Power: %d dBm\n", (int)LORA_PWR_DBM);
  std::printf("[INIT] Ready. Listening for binary packets...\n\n");

  unsigned long lastBeacon     = millis();
  unsigned long lastCachePrune = millis();

  while (true) {
    unsigned long now = millis();

    // Periodic beacon transmission
    if (!isWithin(now, lastBeacon, BEACON_INTERVAL_MS)) {
      transmitBeacon();
      lastBeacon = millis();
    }

    // Periodic dedup cache pruning (every 10s)
    if (!isWithin(now, lastCachePrune, 10000UL)) {
      pruneSeenCache();
      lastCachePrune = millis();
    }

    // RX flow similar to your previous Pi code
    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
      std::printf("[RX] startReceive failed (code=%d)\n", state);
      hal->delay(100);
      continue;
    }

    hal->delay(RX_TIMEOUT_MS);

    uint8_t rxBuf[RX_BUFFER_SIZE];
    state = radio.readData(rxBuf, RX_BUFFER_SIZE);

    if (state == RADIOLIB_ERR_NONE) {
      size_t len = radio.getPacketLength();
      if (len == 0 || len > RX_BUFFER_SIZE) {
        std::printf("[RX] Malformed length (len=%zu)\n", len);
        continue;
      }

      float rssi = radio.getRSSI();
      float snr  = radio.getSNR();
      uint8_t type = rxBuf[0];

      switch (type) {
        case PKT_BEACON: {
          if (len < sizeof(BeaconPacket)) {
            std::printf("[RX] Beacon too short (len=%zu)\n", len);
            break;
          }
          BeaconPacket bp;
          std::memcpy(&bp, rxBuf, sizeof(BeaconPacket));
          uint8_t calc = crc8((uint8_t*)&bp, sizeof(bp) - 1);
          if (calc != bp.crc) {
            std::printf("[CRC] Beacon CRC error\n");
            break;
          }
          handleBeaconPacket(bp, rssi, snr);
          break;
        }

        case PKT_DATA: {
          size_t headerSize = offsetof(DataPacket, payload);
          if (len < headerSize + 1) {
            std::printf("[RX] Data too short (header)\n");
            break;
          }

          DataPacket dp;
          std::memset(&dp, 0, sizeof(dp));
          std::memcpy(&dp, rxBuf, headerSize);  // header + payloadLen

          if (dp.payloadLen > MAX_PAYLOAD_LEN) {
            std::printf("[RX] Data payload too large (%u)\n", (unsigned)dp.payloadLen);
            break;
          }

          size_t needed = headerSize + dp.payloadLen + 1; // + crc
          if (len < needed) {
            std::printf("[RX] Data length mismatch (len=%zu needed=%zu)\n",
                        len, needed);
            break;
          }

          std::memcpy(dp.payload, rxBuf + headerSize, dp.payloadLen);
          dp.crc = rxBuf[headerSize + dp.payloadLen];

          uint8_t calc = crc8((uint8_t*)&dp, headerSize + dp.payloadLen);
          if (calc != dp.crc) {
            std::printf("[CRC] Data CRC error\n");
            break;
          }

          handleDataPacket(dp, rssi, snr);
          break;
        }

        case PKT_ACK: {
          if (len < sizeof(AckPacket)) {
            std::printf("[RX] Ack too short (len=%zu)\n", len);
            break;
          }
          AckPacket ap;
          std::memcpy(&ap, rxBuf, sizeof(AckPacket));
          uint8_t calc = crc8((uint8_t*)&ap, sizeof(ap) - 1);
          if (calc != ap.crc) {
            std::printf("[CRC] Ack CRC error\n");
            break;
          }
          // We don't need ACK for logic here, just log
          std::printf("[RX] ACK src=%u dst=%u ackSeq=%u\n",
                      (unsigned)ap.srcId,
                      (unsigned)ap.dstId,
                      (unsigned)ap.ackSeq);
          break;
        }

        case PKT_JOIN_REQ: {
          if (len < sizeof(JoinReqPacket)) {
            std::printf("[RX] JoinReq too short (len=%zu)\n", len);
            break;
          }
          JoinReqPacket jp;
          std::memcpy(&jp, rxBuf, sizeof(JoinReqPacket));
          uint8_t calc = crc8((uint8_t*)&jp, sizeof(jp) - 1);
          if (calc != jp.crc) {
            std::printf("[CRC] JoinReq CRC error\n");
            break;
          }
          handleJoinReqPacket(jp);
          break;
        }

        case PKT_JOIN_ACK: {
          if (len < sizeof(JoinAckPacket)) {
            std::printf("[RX] JoinAck too short (len=%zu)\n", len);
            break;
          }
          JoinAckPacket ja;
          std::memcpy(&ja, rxBuf, sizeof(JoinAckPacket));
          uint8_t calc = crc8((uint8_t*)&ja, sizeof(ja) - 1);
          if (calc != ja.crc) {
            std::printf("[CRC] JoinAck CRC error\n");
            break;
          }
          handleJoinAckPacket(ja);
          break;
        }

        default:
          std::printf("[RX] Unknown type %u (len=%zu)\n",
                      (unsigned)type, len);
          break;
      }

    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      // normal
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      std::printf("[RX] PHY CRC mismatch\n");
    } else {
      std::printf("[RX] Receive failed (code=%d)\n", state);
    }
  }

  return 0;
}

