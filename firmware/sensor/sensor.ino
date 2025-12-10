#include <Arduino.h>
#include <RadioLib.h>
#include <stddef.h>

// =============================================================
//  DEMO SENSOR NODE (ACTIVE PROBE DISCOVERY + JOIN HANDSHAKE)
//  UPDATED: OPTIMIZED TIMING (Solution 1)
// =============================================================

// ================== Radio / LoRa Config ==================

static const float   FREQ_MHZ   = 923.2f; // must match repeater
static const float   BW_KHZ     = 500.0f;
static const uint8_t SF         = 5;
static const uint8_t CR         = 5;      // 4/5
static const uint8_t SYNCWORD   = 0x12;
static const int8_t  PWR_DBM    = -5;     // same as repeater
static const uint16_t PREAMBLE  = 8;
static const float   TCXO_V     = 1.6f;
static const bool    USE_LDO    = false;

// ================== Mesh / Node Config ==================

// IMPORTANT: Pick ID that is NOT 0 (sink), 1, or 2 (your repeaters)
static const uint8_t DEVICE_ID  = 5;      // demo sensor node ID
static const uint8_t SINK_ID    = 0;      // sink destination
static const uint8_t MAX_TTL    = 3;      // initial TTL for DATA

// Timing
static const unsigned long DISCOVERY_LOG_INTERVAL   = 10000UL; // 10s log
static const unsigned long PROBE_INTERVAL           = 3000UL; // 3s between probes
static const unsigned long MIN_DISCOVERY_TIME       = 3000UL; // wait at least 3s
static const unsigned long FULL_DISCOVERY_TIME      = 8000UL; // after 8s, accept even 1 neighbor
static const unsigned long BEACON_TIMEOUT           = 120000UL; // 2min (kept for neighbor cleanup)
static const unsigned long ACK_TIMEOUT              = 5000UL; // 5s wait for ACK
static const unsigned long UPLINK_INTERVAL          = 15000UL; // 15s data send
static const unsigned long JOIN_REQ_TIMEOUT         = 7000UL; // 7s (matches repeater)

static const uint8_t MAX_DATA_RETRIES   = 3;

// Neighbor selection
static const int RSSI_THRESHOLD         = -120;   // dBm (matches repeater)
static const int RANK_PREFERENCE_WEIGHT = 10;     // dB per hop
static const uint8_t MAX_NEIGHBORS      = 10;

// ================== Radio Instance (Wio-E5 Mini) ==================

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

// ================== Binary Mesh Protocol ==================

enum PacketType : uint8_t {
  PKT_BEACON     = 0,
  PKT_DATA       = 1,
  PKT_ACK        = 2,
  PKT_JOIN_REQ   = 3,
  PKT_JOIN_ACK   = 4,
  PKT_PROBE_REQ  = 5,   // NEW: sensor -> repeaters
  PKT_PROBE_RESP = 6    // NEW: repeaters -> sensor
};

static const size_t MAX_PAYLOAD_LEN = 64;
static const size_t RX_BUFFER_SIZE  = 128;

struct __attribute__((packed)) BeaconPacket {
  uint8_t  type;   // PKT_BEACON
  uint8_t  srcId;
  uint8_t  rank;
  uint16_t seq;
  uint8_t  crc;
};

struct __attribute__((packed)) DataPacket {
  uint8_t  type;     // PKT_DATA
  uint8_t  srcId;    // immediate sender (this sensor)
  uint8_t  dstId;    // parent repeater or sink
  uint8_t  ttl;
  uint8_t  origSrc;  // original sensor ID
  uint16_t origSeq;  // original sensor sequence
  uint16_t seq;      // local seq (matches origSeq here)
  uint8_t  payloadLen;
  uint8_t  payload[MAX_PAYLOAD_LEN];
  uint8_t  crc;
};

struct __attribute__((packed)) AckPacket {
  uint8_t  type;   // PKT_ACK
  uint8_t  srcId;  // parent ID (repeater/sink)
  uint8_t  dstId;  // DEVICE_ID
  uint16_t ackSeq; // original sensor sequence (origSeq)
  uint8_t  crc;
};

struct __attribute__((packed)) JoinReqPacket {
  uint8_t  type;     // PKT_JOIN_REQ
  uint8_t  srcId;    // DEVICE_ID
  uint8_t  parentId; // chosen repeater
  uint16_t seq;
  uint8_t  crc;
};

struct __attribute__((packed)) JoinAckPacket {
  uint8_t  type;     // PKT_JOIN_ACK
  uint8_t  srcId;    // parent ID
  uint8_t  childId;  // DEVICE_ID
  uint16_t seq;      // echo of JOIN_REQ seq
  uint8_t  crc;
};

// active probe packets
struct __attribute__((packed)) ProbeReqPacket {
  uint8_t  type;   // PKT_PROBE_REQ
  uint8_t  srcId;  // sensor ID
  uint16_t seq;
  uint8_t  crc;
};

struct __attribute__((packed)) ProbeRespPacket {
  uint8_t  type;   // PKT_PROBE_RESP
  uint8_t  srcId;  // repeater ID
  uint8_t  rank;   // repeater rank
  uint16_t seq;    // echo probe seq
  uint8_t  crc;
};

// Binary Sensor Payload (14 bytes total)
struct __attribute__((packed)) SensorPayload {
  uint8_t  nodeId;      // 1 byte
  uint32_t counter;     // 4 bytes
  int16_t  tempAir;     // 2 bytes (multiplied by 100)
  int16_t  tempSoil;    // 2 bytes (multiplied by 100)
  uint16_t humidity;    // 2 bytes (multiplied by 100)
  uint16_t lux;         // 2 bytes
  uint8_t  moisture;    // 1 byte
};

// ================== Neighbor Table Structure ==================

struct Neighbor {
  int           id;
  uint8_t       rank;
  int           rssi;
  unsigned long lastHeard;
  bool          isValid;
};

// ================== State Variables ==================

Neighbor neighborTable[MAX_NEIGHBORS];

enum SensorState {
  DISCOVERING,
  JOINING,
  JOINED
};

SensorState currentState = DISCOVERING;
int      bestRepeaterId   = -1;
uint8_t  bestRepeaterRank = 255;
bool     parentConfirmed  = false;   // once JOIN is complete

unsigned long bootTime           = 0;
unsigned long lastDiscoveryLog   = 0;
unsigned long lastUplinkTime     = 0;
unsigned long joinRequestSent    = 0;

// PROBE state
uint16_t      probeSequence   = 0;
unsigned long lastProbeSent   = 0;
unsigned long childCount = 0;

// ================== Uplink State ==================

struct UplinkState {
  bool          active   = false;
  uint16_t      seq      = 0;
  
  // Store raw bytes instead of String
  uint8_t       payloadBlob[MAX_PAYLOAD_LEN]; 
  uint8_t       payloadLen = 0;
  
  uint8_t       retries  = 0;
  unsigned long lastSend = 0;
};

UplinkState uplinkState;

uint16_t uplinkSequence = 0;
uint16_t joinSequence   = 0; // separate sequence for JOIN_REQ

// ================== CRC Function ==================

// Simple CRC-8 (poly 0x07, init 0x00)
uint8_t crc8(const uint8_t* data, size_t len) {
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

// ================== Utility Functions ==================

uint16_t nextUplinkSeq() {
  return ++uplinkSequence;
}

uint16_t nextJoinSeq() {
  return ++joinSequence;
}

uint16_t nextProbeSeq() {
  return ++probeSequence;
}

bool isWithin(unsigned long now, unsigned long start, unsigned long interval) {
  if (start > now) return true; 
  return (now - start) < interval;
}

void clearNeighborTable() {
  for (uint8_t i = 0; i < MAX_NEIGHBORS; i++) {
    neighborTable[i].id        = -1;
    neighborTable[i].rank      = 255;
    neighborTable[i].rssi      = -127;
    neighborTable[i].lastHeard = 0;
    neighborTable[i].isValid   = false;
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

  slot->id        = nodeId;
  slot->rank      = rank;
  slot->rssi      = rssi;
  slot->lastHeard = now;
  slot->isValid   = true;

  Serial.print(F("[NB] "));
  Serial.print(nodeId);
  Serial.print(F(" rank="));
  Serial.print(rank);
  Serial.print(F(" rssi="));
  Serial.println(rssi);
}

void pruneStaleNeighbors(unsigned long now) {
  for (uint8_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighborTable[i].isValid && !isWithin(now, neighborTable[i].lastHeard, BEACON_TIMEOUT)) {
      Serial.print(F("[NEIGHBOR] Timeout -> "));
      Serial.println(neighborTable[i].id);
      neighborTable[i].isValid = false;
    }
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

// ================== Dummy Payload ==================

void generateBinaryPayload(uint8_t* buffer, uint8_t& len) {
  static uint32_t counter = 0;
  counter++;

  // Generate dummy values
  float rawAirTemp   = 20.0f + (counter % 50) * 0.1f;
  float rawSoilTemp  = 18.0f + (counter % 40) * 0.1f;
  float rawHumidity  = 40.0f + (counter % 60) * 0.5f;
  float rawLux       = 100.0f + (counter % 200) * 1.0f;
  int   rawMoisture  = (counter * 7) % 100;

  // Pack into struct
  SensorPayload sp;
  sp.nodeId   = DEVICE_ID;
  sp.counter  = counter;
  
  // Convert floats to integers (x100 preserves 2 decimal places)
  sp.tempAir  = (int16_t)(rawAirTemp * 100);
  sp.tempSoil = (int16_t)(rawSoilTemp * 100);
  sp.humidity = (uint16_t)(rawHumidity * 100);
  sp.lux      = (uint16_t)rawLux;
  sp.moisture = (uint8_t)rawMoisture;

  // Copy struct to buffer
  len = sizeof(SensorPayload);
  memcpy(buffer, &sp, len);
}

// ================== Parent Selection ==================

bool hasCollectedEnoughNeighbors(unsigned long now) {
  unsigned long elapsed = now - bootTime;
  uint8_t n = countValidNeighbors();

  if (n == 0) return false;
  if (elapsed < MIN_DISCOVERY_TIME) return false;

  // If we already saw a rank-1 neighbor with decent RSSI, that's enough quickly
  bool hasGoodRank1 = false;
  for (uint8_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighborTable[i].isValid && neighborTable[i].rank == 1) {
      if (neighborTable[i].rssi > RSSI_THRESHOLD + 10) {
        hasGoodRank1 = true;
        break;
      }
    }
  }
  if (hasGoodRank1) return true;

  // If we have 2+ neighbors, that's enough
  if (n >= 2) return true;

  // After FULL_DISCOVERY_TIME, it's OK to decide with even 1 neighbor
  if (elapsed >= FULL_DISCOVERY_TIME && n >= 1) return true;

  return false;
}

int selectBestRepeater() {
  Neighbor* best = nullptr;
  int bestScore = -999999;

  for (uint8_t i = 0; i < MAX_NEIGHBORS; i++) {
    Neighbor& n = neighborTable[i];
    if (!n.isValid) continue;
    if (n.rank >= 255) continue;        // ignore unjoined repeaters
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

// ================== JOIN Handshake ==================

void sendJoinRequest() {
  if (bestRepeaterId == -1) return;

  // Direct sink needs no handshake
  if (bestRepeaterId == SINK_ID) {
    parentConfirmed = true;
    currentState    = JOINED;
    Serial.println(F("[JOIN] Direct sink connection, no JOIN_REQ needed"));
    return;
  }

  JoinReqPacket pkt;
  pkt.type     = PKT_JOIN_REQ;
  pkt.srcId    = DEVICE_ID;
  pkt.parentId = (uint8_t)bestRepeaterId;
  pkt.seq      = nextJoinSeq();
  pkt.crc      = crc8((uint8_t*)&pkt, sizeof(pkt) - 1);

  int16_t state = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
  
  if (state == RADIOLIB_ERR_NONE) {
    joinRequestSent = millis();
    
    // ==========================================================
    // CRITICAL TIMING FIX (Solution 1)
    // ==========================================================
    // We do NOT print anything here. Printing at 115200 baud takes 
    // ~2.5ms for a short string. At SF5/BW500, the preamble is 
    // only ~0.5ms. If we print, we miss the Repeater's ACK.
    // ==========================================================
    
  } else {
    Serial.print(F("[JOIN][ERR] TX failed: "));
    Serial.println(state);
  }
}

void handleJoinAckPacket(const JoinAckPacket& pkt) {
  if (pkt.childId != DEVICE_ID) return;
  if (currentState != JOINING) return;

  Serial.print(F("[JOIN] JOIN_ACK received from "));
  Serial.print(pkt.srcId);
  Serial.print(F(" seq="));
  Serial.println(pkt.seq);

  // Verify parent
  if (pkt.srcId != bestRepeaterId) {
    Serial.println(F("[JOIN] JOIN_ACK from unexpected parent, ignoring"));
    return;
  }

  parentConfirmed = true;
  currentState    = JOINED;

  Serial.print(F("[JOIN] Confirmed parent "));
  Serial.print(bestRepeaterId);
  Serial.print(F(" rank="));
  Serial.println(bestRepeaterRank);

  lastUplinkTime = millis(); // resets uplink timer to not immediately send data

  // ==========================================================
  // CRITICAL TIMING FIX (Solution 1)
  // ==========================================================
  // Give the Repeater time to switch from TX to RX. 
  // Without this delay, we might send the Final ACK before the 
  // Repeater is listening.
  delay(50); 
  // ==========================================================

  // Send final ACK to parent
  AckPacket ack;
  ack.type   = PKT_ACK;
  ack.srcId  = DEVICE_ID;
  ack.dstId  = pkt.srcId;
  ack.ackSeq = pkt.seq;
  ack.crc    = crc8((uint8_t*)&ack, sizeof(ack) - 1);

  radio.transmit((uint8_t*)&ack, sizeof(ack));
  
  Serial.println(F("[JOIN] Final ACK sent to parent"));
}

// ================== PROBE Logic ==================

void sendProbeRequest() {
  ProbeReqPacket pkt;
  pkt.type  = PKT_PROBE_REQ;
  pkt.srcId = DEVICE_ID;
  pkt.seq   = nextProbeSeq();
  pkt.crc   = crc8((uint8_t*)&pkt, sizeof(pkt) - 1);

  int16_t st = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
  if (st == RADIOLIB_ERR_NONE) {
    lastProbeSent = millis();
    Serial.print(F("[PROBE] Sent PROBE_REQ seq="));
    Serial.println(pkt.seq);
  } else {
    Serial.print(F("[PROBE][ERR] TX code="));
    Serial.println(st);
  }
}

void handleProbeRespPacket(const ProbeRespPacket& pkt, float rssi, unsigned long now) {
  // Store repeater as neighbor candidate
  updateNeighbor(pkt.srcId, pkt.rank, (int)rssi, now);

  Serial.print(F("[PROBE] RESP from "));
  Serial.print(pkt.srcId);
  Serial.print(F(" rank="));
  Serial.print(pkt.rank);
  Serial.print(F(" rssi="));
  Serial.println(rssi);
}

// ================== Uplink Logic ==================

void startNewUplink() {
  uplinkState.active  = true;
  uplinkState.seq     = nextUplinkSeq();
  
  // Generate the binary payload once and store it
  generateBinaryPayload(uplinkState.payloadBlob, uplinkState.payloadLen);
  
  uplinkState.retries = 0;

  Serial.print(F("[UPLINK] Start seq="));
  Serial.print(uplinkState.seq);
  Serial.print(F(" len="));
  Serial.println(uplinkState.payloadLen);
}

void transmitUplink() {
  if (!uplinkState.active) return;

  if (bestRepeaterId < 0) {
    Serial.println(F("[UPLINK] No parent, abort"));
    uplinkState.active = false;
    return;
  }

  DataPacket pkt;
  memset(&pkt, 0, sizeof(pkt)); // clears memory

  pkt.type    = PKT_DATA;
  pkt.srcId   = DEVICE_ID;
  pkt.dstId   = SINK_ID; // Always route to sink (repeater will forward)
  pkt.ttl     = MAX_TTL;
  pkt.origSrc = DEVICE_ID;
  pkt.origSeq = uplinkState.seq;
  pkt.seq     = uplinkState.seq; // local seq (matches origSeq here)

  // Copy payload to packet
  pkt.payloadLen = uplinkState.payloadLen;
  memcpy(pkt.payload, uplinkState.payloadBlob, uplinkState.payloadLen);

  // Calculate CRC over header + payload
  size_t headerSize = offsetof(DataPacket, payload);
  uint8_t calculatedCRC = crc8((uint8_t*)&pkt, headerSize + pkt.payloadLen);

  // Write CRC to the position immediately following the payload
  pkt.payload[pkt.payloadLen] = calculatedCRC; // actual pkt.crc field ignored bcs out of transmission range

  size_t totalSize = headerSize + pkt.payloadLen + 1; // +1 for CRC

  int16_t state = radio.transmit((uint8_t*)&pkt, totalSize);
  
  if (state == RADIOLIB_ERR_NONE) {
    uplinkState.lastSend = millis();
    uplinkState.retries++;
    delay(5); // Safety delay for stability
    Serial.print(F("[UPLINK] Sent DATA seq="));
    Serial.println(uplinkState.seq);
  } else {
    Serial.print(F("[UPLINK][ERR] TX code="));
    Serial.println(state);
  }
}

void attemptRepeaterFailover(unsigned long now); // forward decl

void handleUplinkTimeout(unsigned long now) {
  if (!uplinkState.active) return;

  if (uplinkState.retries < MAX_DATA_RETRIES) {
    Serial.println(F("[UPLINK] No ACK, retry DATA"));
    transmitUplink();
  } else {
    Serial.println(F("[UPLINK] Max retries, parent may be dead"));
    uplinkState.active = false;
    
    Neighbor* n = findNeighbor(bestRepeaterId);
    if (n) n->isValid = false;
    
    attemptRepeaterFailover(now);
  }
}

// ================== Packet Handlers ==================

void handleBeaconPacket(const BeaconPacket& pkt, float rssi, unsigned long now) {
  // Always update neighbor info, even rank=255 neighbors
  // (We might want to track them for future fallback)
  updateNeighbor(pkt.srcId, pkt.rank, (int)rssi, now);

  Serial.print(F("[BEACON] From "));
  Serial.print(pkt.srcId);
  Serial.print(F(" rank="));
  Serial.print(pkt.rank);
  Serial.print(F(" rssi="));
  Serial.println(rssi);
}

void handleAckPacket(const AckPacket& pkt) {
  if (!uplinkState.active) return;
  if (pkt.dstId != DEVICE_ID) return;
  if (pkt.ackSeq != uplinkState.seq) return;

  Serial.print(F("[ACK] Received for seq="));
  Serial.println(uplinkState.seq);

  uplinkState.active = false;
  Serial.println(F("[UPLINK] Success"));
}

// ================== Failover ==================

bool isRepeaterStillAlive(int repeaterId, unsigned long now) {
  if (repeaterId == -1) return false;
  Neighbor* n = findNeighbor(repeaterId);
  if (!n || !n->isValid) return false;
  if (!isWithin(now, n->lastHeard, BEACON_TIMEOUT)) return false;
  return true;
}

void attemptRepeaterFailover(unsigned long now) {
  Serial.println(F("[FAILOVER] Attempting backup repeater"));
  pruneStaleNeighbors(now);

  int newRepeater = selectBestRepeater();

  if (newRepeater != -1 && newRepeater != bestRepeaterId) {
    bestRepeaterId   = newRepeater;
    parentConfirmed  = false;  // Need new JOIN handshake
    currentState     = JOINING;
    Serial.print(F("[FAILOVER] Switching to repeater "));
    Serial.println(bestRepeaterId);
    sendJoinRequest();
  } else {
    Serial.println(F("[FAILOVER] No backup found, restarting discovery"));
    bestRepeaterId     = -1;
    parentConfirmed    = false;
    currentState       = DISCOVERING;
    uplinkState.active = false;
    clearNeighborTable();
    bootTime        = now;
    lastProbeSent   = 0;
  }
}

// ================== Radio Polling ==================

void pollRadio() {
  for (int i = 0; i < 5; i++) {
    uint8_t rxBuf[RX_BUFFER_SIZE];
    int16_t rxState = radio.receive(rxBuf, RX_BUFFER_SIZE);

    if (rxState == RADIOLIB_ERR_NONE) {
      size_t len = radio.getPacketLength();
      if (len == 0 || len > RX_BUFFER_SIZE) {
        Serial.println(F("[RX] Malformed length"));
        continue;
      }

      float rssi = radio.getRSSI();
      uint8_t type = rxBuf[0];

      switch (type) {
        case PKT_BEACON: {
          if (len < sizeof(BeaconPacket)) break;
          BeaconPacket bp;
          memcpy(&bp, rxBuf, sizeof(BeaconPacket));
          uint8_t calc = crc8((uint8_t*)&bp, sizeof(bp) - 1);
          if (calc != bp.crc) {
            Serial.println(F("[CRC] Beacon error"));
            break;
          }
          handleBeaconPacket(bp, rssi, millis());
          break;
        }

        case PKT_ACK: {
          if (len < sizeof(AckPacket)) break;
          AckPacket ap;
          memcpy(&ap, rxBuf, sizeof(AckPacket));
          uint8_t calc = crc8((uint8_t*)&ap, sizeof(ap) - 1);
          if (calc != ap.crc) {
            Serial.println(F("[CRC] Ack error"));
            break;
          }
          handleAckPacket(ap);
          break;
        }

        case PKT_JOIN_ACK: {
          if (len < sizeof(JoinAckPacket)) break;
          JoinAckPacket ja;
          memcpy(&ja, rxBuf, sizeof(JoinAckPacket));
          uint8_t calc = crc8((uint8_t*)&ja, sizeof(ja) - 1);
          if (calc != ja.crc) {
            Serial.println(F("[CRC] JoinAck error"));
            break;
          }
          handleJoinAckPacket(ja);
          break;
        }

        case PKT_PROBE_RESP: {
          if (len < sizeof(ProbeRespPacket)) break;
          ProbeRespPacket pr;
          memcpy(&pr, rxBuf, sizeof(ProbeRespPacket));
          uint8_t calc = crc8((uint8_t*)&pr, sizeof(pr) - 1);
          if (calc != pr.crc) {
            Serial.println(F("[CRC] ProbeResp error"));
            break;
          }
          handleProbeRespPacket(pr, rssi, millis());
          break;
        }

        case PKT_PROBE_REQ:
          // Sensor ignores other sensors' PROBE_REQ
          break;

        default:
          break;
      }

    } else if (rxState == RADIOLIB_ERR_RX_TIMEOUT ||
               rxState == RADIOLIB_ERR_CRC_MISMATCH) {
      break;
    } else {
      Serial.print(F("[RX][ERR] "));
      Serial.println(rxState);
      break;
    }
  }
}

// ================== State Handlers ==================

void handleDiscoveryState(unsigned long now) {
  pruneStaleNeighbors(now);

  // Periodically send PROBE_REQ to wake up repeaters and get fresh rank/RSSI
  if (lastProbeSent == 0 || (now - lastProbeSent) >= PROBE_INTERVAL) {
    sendProbeRequest();
  }

  if (!isWithin(now, lastDiscoveryLog, DISCOVERY_LOG_INTERVAL)) {
    Serial.print(F("[DISCOVERY] neighbors="));
    Serial.println(countValidNeighbors());
    lastDiscoveryLog = now;
  }

  if (hasCollectedEnoughNeighbors(now)) {
    bestRepeaterId = selectBestRepeater();
    if (bestRepeaterId != -1) {
      Serial.print(F("[DISCOVERY] Selected repeater "));
      Serial.print(bestRepeaterId);
      Serial.print(F(" (rank="));
      Serial.print(bestRepeaterRank);
      Serial.println(')');
      
      currentState = JOINING;
      sendJoinRequest();
    } else {
      Serial.println(F("[DISCOVERY] No suitable repeater yet"));
    }
  }
}

void handleJoiningState(unsigned long now) {
  // Wait for JOIN_ACK or timeout
  if (!isWithin(now, joinRequestSent, JOIN_REQ_TIMEOUT)) {
    Serial.println(F("[JOIN] Timeout, retrying"));
    sendJoinRequest();
  }
  
  pruneStaleNeighbors(now);
}

void handleJoinedState(unsigned long now) {
  // Periodic uplink

  bool packetJustSent = false;

  if (!uplinkState.active) {
    if (!isWithin(now, lastUplinkTime, UPLINK_INTERVAL)) {
      startNewUplink();
      transmitUplink();
      lastUplinkTime = now;
      packetJustSent = true; // Mark that we just transmitted
    }
  }

  // Wait for ACKs / handle timeout
  if (!packetJustSent && uplinkState.active && !isWithin(now, uplinkState.lastSend, ACK_TIMEOUT)) {
    handleUplinkTimeout(now);
  }
}

// ================== Setup/Loop ==================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("=== Demo Sensor Node (Active Probe) ==="));
  Serial.print(F("NODE_ID="));
  Serial.println(DEVICE_ID);

  bootTime = millis();
  randomSeed(analogRead(A0));

  radio.setRfSwitchTable(RFSW_PINS, RFSW_TABLE);
  Serial.println(F("[INIT] RF switch table applied"));

  int16_t stateInit = radio.begin(
    FREQ_MHZ, BW_KHZ, SF, CR, SYNCWORD, PWR_DBM, PREAMBLE, TCXO_V, USE_LDO
  );

  Serial.print(F("[INIT] radio.begin -> "));
  Serial.println(stateInit);

  if (stateInit != RADIOLIB_ERR_NONE) {
    Serial.println(F("[FATAL] LoRa init failed"));
    while (true) delay(1000);
  }

  radio.setCRC(true);
  clearNeighborTable();
  
  uplinkState.active   = false;
  uplinkState.seq      = 0;
  uplinkState.retries  = 0;
  uplinkState.lastSend = 0;

  lastProbeSent = 0;

  Serial.println(F("[INIT] Discovery started"));
}

void loop() {
  pollRadio();
  
  unsigned long now = millis();

  switch (currentState) {
    case DISCOVERING:
      handleDiscoveryState(now);
      break;

    case JOINING:
      handleJoiningState(now);
      break;

    case JOINED:
      handleJoinedState(now);
      break;
  }
}
