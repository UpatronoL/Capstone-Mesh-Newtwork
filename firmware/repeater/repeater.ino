#include <Arduino.h>
#include <RadioLib.h>
#include <stddef.h>  // for offsetof

// =============================================================
//  SIMPLE BINARY REPEATER (WITH CRC + JOIN HANDSHAKE + PROBE RESP)
// =============================================================

STM32WLx_Module wl;
STM32WLx radio(&wl);

// ---------------- Radio config ----------------

static const uint32_t RFSW_PINS[] = {
  PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
};
static const Module::RfSwitchMode_t RFSW_TABLE[] = {
  { STM32WLx::MODE_IDLE,  { LOW, LOW } },
  { STM32WLx::MODE_RX,    { HIGH, LOW } },
  { STM32WLx::MODE_TX_LP, { LOW, HIGH } },
  { STM32WLx::MODE_TX_HP, { LOW, HIGH } },
  END_OF_MODE_TABLE,
};
static const float   FREQ_MHZ  = 923.2f;
static const float   BW_KHZ    = 500.0f;
static const uint8_t SF        = 5;
static const uint8_t CR        = 5;
static const uint8_t SYNCWORD  = 0x12;
static const int8_t  PWR_DBM   = -5;
static const uint16_t PREAMBLE = 8;
static const float   TCXO_V    = 1.6f;
static const bool    USE_LDO   = false;

// ---------------- IDs & constants ----------------

// CHANGE THIS PER NODE (e.g. 1 for rep1, 2 for rep2)
static const uint8_t NODE_ID       = 2;
static const uint8_t SINK_ID       = 0;
static const uint8_t DST_BROADCAST = 255;  // special "to sink / broadcast" dest

enum PacketType : uint8_t {
  PKT_BEACON     = 0,
  PKT_DATA       = 1,
  PKT_ACK        = 2,
  PKT_JOIN_REQ   = 3,
  PKT_JOIN_ACK   = 4,
  PKT_PROBE_REQ  = 5,   // NEW: sensor→repeaters
  PKT_PROBE_RESP = 6    // NEW: repeaters→sensor
};

enum RepeaterMode {
  MODE_BOOT_LISTEN,
  MODE_SEARCHING,
  MODE_STEADY,
  MODE_REPAIR,
  MODE_JOIN_HANDLING
};

RepeaterMode mode = MODE_BOOT_LISTEN;

// Radio busy flag (for TX)
bool radioBusy = false;

// ---------------- Timings ----------------

static const unsigned long BOOT_LISTEN_DURATION      = 15000UL;
static const unsigned long SEARCH_BEACON_INTERVAL    = 5000UL;
static const unsigned long SEARCH_BEACON_JITTER      = 3000UL;
static const unsigned long STEADY_BEACON_INTERVAL    = 30000UL;
static const unsigned long NEIGHBOR_TIMEOUT          = 120000UL;
static const unsigned long DEDUP_TIMEOUT             = 30000UL;
static const unsigned long ACK_TIMEOUT               = 3000UL;
static const unsigned long SEARCH_WINDOW_MS          = 5000UL;
static const unsigned long JOIN_REQ_TIMEOUT          = 7000UL;
static const unsigned long PARENT_SANITY_INTERVAL    = 60000UL;

// Steady-state housekeeping
static const unsigned long HOUSEKEEPING_INTERVAL = 300000UL;  // 5 minutes
unsigned long lastHousekeeping = 0;

// JOIN handshake retry timing
static const unsigned long JOIN_ACK_RETRY_INTERVAL = 1000UL;  // 1s between JOIN_ACKs
static const uint8_t JOIN_ACK_MAX_RETRIES          = 5;

// NEW: proactive join-assist timeout
static const unsigned long JOIN_ASSIST_TIMEOUT = 30000UL;  // 30s to babysit a rank-255 neighbor
unsigned long joinAssistStart = 0;

// ---------------- RSSI / routing thresholds ----------------

static const int16_t RSSI_THRESHOLD                = -120;
static const int16_t DIRECT_RSSI_THRESHOLD         = -100;
static const int16_t PARENT_RSSI_SWITCH_HYSTERESIS = 8;
static const int16_t RSSI_TIE_BREAK_MARGIN         = 5;
static const uint8_t MAX_RETRIES                   = 3;
static const uint8_t MAX_TTL                       = 10;
static const uint8_t PARENT_CONFIRM_COUNT          = 3;

// ---------------- Packet structs (binary + CRC) ----------------

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
  uint8_t  type;  // PKT_BEACON
  uint8_t  srcId;
  uint8_t  rank;
  uint16_t seq;
  uint8_t  crc;
};

struct __attribute__((packed)) DataPacket {
  uint8_t  type;  // PKT_DATA
  uint8_t  srcId;
  uint8_t  dstId;
  uint8_t  ttl;
  uint8_t  origSrc;
  uint16_t origSeq;
  uint16_t seq;
  uint8_t  payloadLen;
  uint8_t  payload[MAX_PAYLOAD_LEN];  // variable portion
  uint8_t  crc;
};

struct __attribute__((packed)) AckPacket {
  uint8_t  type;  // PKT_ACK
  uint8_t  srcId;
  uint8_t  dstId;
  uint16_t ackSeq;
  uint8_t  crc;
};

struct __attribute__((packed)) JoinReqPacket {
  uint8_t  type;  // PKT_JOIN_REQ
  uint8_t  srcId;
  uint8_t  parentId;
  uint16_t seq;
  uint8_t  crc;
};

struct __attribute__((packed)) JoinAckPacket {
  uint8_t  type;   // PKT_JOIN_ACK
  uint8_t  srcId;  // parent ID
  uint8_t  childId;
  uint16_t seq;    // echo of JOIN_REQ seq
  uint8_t  crc;
};

// NEW: active discovery probe packets
struct __attribute__((packed)) ProbeReqPacket {
  uint8_t  type;    // PKT_PROBE_REQ
  uint8_t  srcId;   // sensor ID
  uint16_t seq;     // probe sequence
  uint8_t  crc;
};

struct __attribute__((packed)) ProbeRespPacket {
  uint8_t  type;    // PKT_PROBE_RESP
  uint8_t  srcId;   // repeater ID
  uint8_t  rank;    // repeater's current rank
  uint16_t seq;     // echo of probe seq
  uint8_t  crc;
};

// ---------------- State / tables ----------------

uint16_t beaconSeq = 0;
uint16_t dataSeq   = 0;

int  myRank          = 255;
int  bestNextHop     = -1;
bool parentConfirmed = false;
bool awaitingJoinAck = false;
unsigned long joinRequestSent = 0;

unsigned long bootListenStart   = 0;
unsigned long searchWindowStart = 0;
unsigned long nextBeaconAt      = 0;
unsigned long lastParentSanity  = 0;

// Parent confirmation (for switching parents)
int     candidateParent = -1;
uint8_t candidateCount  = 0;

// JOIN handshake state (parent side)
uint8_t       pendingChildId   = 0;
uint16_t      pendingJoinSeq   = 0;
unsigned long lastJoinAckSent  = 0;
uint8_t       joinAckRetries   = 0;

// Neighbors
struct Neighbor {
  int           id        = -1;
  int           rank      = 255;
  int           rssi      = -200;
  unsigned long lastHeard = 0;
  bool          isValid   = false;
};

static const size_t MAX_NEIGHBORS = 8;
Neighbor neighbors[MAX_NEIGHBORS];

// Probes
struct Probe {
  int           id        = -1;
  int           rssi      = -200;
  unsigned long lastHeard = 0;
  bool          isValid   = false;
};

static const size_t MAX_PROBES = 8; // same as neighbor for now
Probe probes[MAX_PROBES];

// Dedup cache
struct SeenPacket {
  uint8_t       src   = 255;
  uint16_t      seq   = 0;
  unsigned long ts    = 0;
  bool          valid = false;
};

static const size_t CACHE_SIZE = 20;
SeenPacket seen[CACHE_SIZE];

// Forward queue (single packet)
struct ForwardState {
  bool          active     = false;
  uint8_t       origSrc;
  uint16_t      origSeq;
  uint8_t       ttl;
  uint8_t       payloadLen;
  uint8_t       payload[MAX_PAYLOAD_LEN];
  uint8_t       retries;
  unsigned long lastSend;
};

ForwardState forwardState;

// Stats
struct Stats {
  uint32_t beaconsSent       = 0;
  uint32_t dataForwarded     = 0;
  uint32_t acksReceived      = 0;
  uint32_t duplicatesDropped = 0;
  uint32_t parentSwitches    = 0;

  uint32_t txErrors          = 0;
  uint32_t rxErrors          = 0;
  uint32_t crcErrors         = 0;
  uint32_t malformedPackets  = 0;
  uint32_t neighborTimeouts  = 0;

  uint32_t probeReqsReceived = 0;
  uint32_t successfulJoins   = 0;
} stats;

// RX buffer
uint8_t rxBuf[RX_BUFFER_SIZE];

// =============================================================
//  UTILS
// =============================================================

bool isWithin(unsigned long now, unsigned long start, unsigned long interval) {
  return (now - start) < interval;
}

void transitionTo(RepeaterMode newMode) {
  if (mode == newMode) return;
  Serial.print(F("[MODE] "));
  Serial.print(mode);
  Serial.print(F(" -> "));
  Serial.println(newMode);
  mode = newMode;
}

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

// =============================================================
//  DUPLICATE FILTER
// =============================================================

bool isDuplicate(uint8_t src, uint16_t seq) {
  unsigned long now = millis();
  for (size_t i = 0; i < CACHE_SIZE; i++) {
    if (seen[i].valid && seen[i].src == src && seen[i].seq == seq) {
      if (isWithin(now, seen[i].ts, DEDUP_TIMEOUT)) {
        return true;
      } else {
        seen[i].valid = false;
      }
    }
  }
  return false;
}

void markSeen(uint8_t src, uint16_t seq) {
  unsigned long now = millis();
  size_t        oldestIdx = 0;
  unsigned long oldest    = now;

  for (size_t i = 0; i < CACHE_SIZE; i++) {
    if (!seen[i].valid) {
      oldestIdx = i;
      break;
    }
    if (seen[i].ts < oldest) {
      oldest = seen[i].ts;
      oldestIdx = i;
    }
  }

  seen[oldestIdx].src   = src;
  seen[oldestIdx].seq   = seq;
  seen[oldestIdx].ts    = now;
  seen[oldestIdx].valid = true;
}

// =============================================================
//  NEIGHBORS & PARENT & PROBE
// =============================================================

Neighbor* findNeighbor(int id) {
  for (size_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighbors[i].isValid && neighbors[i].id == id) {
      return &neighbors[i];
    }
  }
  return nullptr;
}

Neighbor* findEmptyNeighbor() {
  for (size_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (!neighbors[i].isValid) return &neighbors[i];
  }
  return nullptr;
}

// Age-aware eviction: score = -rank*1000 + rssi - age_sec
void updateNeighbor(int id, int rank, int rssi, unsigned long now) {
  Neighbor* n = findNeighbor(id);
  if (!n) {
    n = findEmptyNeighbor();
    if (!n) {
      size_t worstIdx   = 0;
      int    worstScore = 0;
      bool   first      = true;
      for (size_t i = 0; i < MAX_NEIGHBORS; i++) {
        if (!neighbors[i].isValid) continue;
        unsigned long ageSec = (now - neighbors[i].lastHeard) / 1000UL;
        int32_t score = -(int32_t)neighbors[i].rank * 1000 +
                        neighbors[i].rssi - (int32_t)ageSec;
        if (first || score < worstScore) {
          worstScore = score;
          worstIdx   = i;
          first      = false;
        }
      }
      if (!first) {
        n = &neighbors[worstIdx];
      }
    }
  }

  if (!n) return;

  n->id        = id;
  n->rank      = rank;
  n->rssi      = rssi;
  n->lastHeard = now;
  n->isValid   = true;

  Serial.print(F("[NB] "));
  Serial.print(id);
  Serial.print(F(" rank="));
  Serial.print(rank);
  Serial.print(F(" rssi="));
  Serial.println(rssi);
}

void pruneNeighbors(unsigned long now) {
  for (size_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighbors[i].isValid && !isWithin(now, neighbors[i].lastHeard, NEIGHBOR_TIMEOUT)) {

      Serial.print(F("[NB] timeout "));
      Serial.println(neighbors[i].id);
      stats.neighborTimeouts++;

      if (neighbors[i].id == bestNextHop) {
        Serial.println(F("[PARENT] Lost parent -> REPAIR"));
        bestNextHop     = -1;
        parentConfirmed = false;
        transitionTo(MODE_REPAIR);
      }
      neighbors[i].isValid = false;
    }
  }
}

int chooseBestParent() {
  int bestId   = -1;
  int bestRank = 255;
  int bestRssi = -999;
  for (size_t i = 0; i < MAX_NEIGHBORS; i++) {
    Neighbor& nb = neighbors[i];
    if (!nb.isValid) continue;
    if (nb.rank == 255) continue;
    if (nb.rssi < RSSI_THRESHOLD) continue;
    if (nb.rank < bestRank ||
        (nb.rank == bestRank && nb.rssi > bestRssi + RSSI_TIE_BREAK_MARGIN)) {
      bestId   = nb.id;
      bestRank = nb.rank;
      bestRssi = nb.rssi;
    }
  }
  return bestId;
}

int chooseBestParentWithHysteresis() {
  int candidate = chooseBestParent();
  if (candidate == -1) return -1;

  if (bestNextHop == -1) return candidate;

  Neighbor* oldP = findNeighbor(bestNextHop);
  Neighbor* newP = findNeighbor(candidate);
  if (!oldP || !oldP->isValid) return candidate;
  if (!newP || !newP->isValid) return bestNextHop;

  if (newP->rank < oldP->rank) return candidate;
  if (newP->rank == oldP->rank &&
      newP->rssi >= oldP->rssi + PARENT_RSSI_SWITCH_HYSTERESIS) {
    return candidate;
  }

  return bestNextHop;
}

void resetParentCandidate() {
  candidateParent = -1;
  candidateCount  = 0;
}

// apply parent immediately
void applyParent(int parent) {
  Neighbor* p = findNeighbor(parent);
  if (!p || !p->isValid || p->rank == 255) {
    Serial.println(F("[RANK] applyParent: invalid parent"));
    myRank          = 255;
    bestNextHop     = -1;
    parentConfirmed = false;
    return;
  }

  int oldParent = bestNextHop;
  myRank        = p->rank + 1;
  bestNextHop   = parent;

  if (oldParent != -1 && oldParent != bestNextHop) {
    stats.parentSwitches++;
    Serial.print(F("[PARENT] Switch "));
    Serial.print(oldParent);
    Serial.print(F(" -> "));
    Serial.println(bestNextHop);
  } else if (oldParent == -1) {
    Serial.print(F("[PARENT] First parent: "));
    Serial.println(bestNextHop);
  }

  Serial.print(F("[RANK] rank="));
  Serial.print(myRank);
  Serial.print(F(" via "));
  Serial.println(bestNextHop);
}

// computeRank: first parent = immediate, later parents = hysteresis
void computeRank() {
  int parent = chooseBestParentWithHysteresis();

  if (parent == -1) {
    if (myRank != 255 || bestNextHop != -1) {
      Serial.println(F("[RANK] No suitable parent"));
    }
    myRank          = 255;
    bestNextHop     = -1;
    parentConfirmed = false;
    resetParentCandidate();
    return;
  }

  // If we're in the middle of JOIN handshake, DON'T change rank/parent
  if (awaitingJoinAck) {
    Serial.println(F("[RANK] Deferring rank change (JOIN in progress)"));
    return;
  }

  // First parent: accept immediately (no candidate delay)
  if (bestNextHop == -1) {
    // Special case: direct sink needs no handshake
    Neighbor* p = findNeighbor(parent);
    if (p && p->id == SINK_ID) {
      applyParent(parent);
      parentConfirmed = true;
      resetParentCandidate();
      return;
    }

    // Normal parent: apply now (but still use JOIN_REQ to confirm later)
    applyParent(parent);
    resetParentCandidate();
    return;
  }

  // Same parent: nothing to do
  if (parent == bestNextHop) {
    resetParentCandidate();
    return;
  }

  // Different candidate: require confirmation before switching
  if (candidateParent != parent) {
    candidateParent = parent;
    candidateCount  = 1;
    Serial.print(F("[PARENT] New candidate "));
    Serial.print(parent);
    Serial.println(F(" (1/x)"));
    return;
  } else {
    candidateCount++;
    Serial.print(F("[PARENT] Confirm "));
    Serial.print(parent);
    Serial.print(F(" ("));
    Serial.print(candidateCount);
    Serial.print(F("/"));
    Serial.print(PARENT_CONFIRM_COUNT);
    Serial.println(F(")"));

    if (candidateCount >= PARENT_CONFIRM_COUNT) {
      applyParent(parent);
      resetParentCandidate();
    }
  }
}

void checkDirectSink(int rssi) {
  if (rssi >= DIRECT_RSSI_THRESHOLD) {
    // Only do something if this is a *change*
    if (myRank != 1 || bestNextHop != SINK_ID || !parentConfirmed) {
      myRank          = 1;
      bestNextHop     = SINK_ID;
      parentConfirmed = true;
      resetParentCandidate();

      Serial.println(F("[RANK] Direct link to sink (rank=1)"));

      // If we’re not already steady, go there now
      if (mode != MODE_STEADY) {
        transitionTo(MODE_STEADY);
        lastHousekeeping = millis();
      }
    }
  }
}

void updateProbes(int id, int rssi, unsigned long now) {
  // 1. Check if probe already exists
  for (size_t i = 0; i < MAX_PROBES; i++) {
    if (probes[i].isValid && probes[i].id == id) {
      probes[i].rssi      = rssi;
      probes[i].lastHeard = now;
      return;
    }
  }

  // 2. Find empty slot
  for (size_t i = 0; i < MAX_PROBES; i++) {
    if (!probes[i].isValid) {
      probes[i].id        = id;
      probes[i].rssi      = rssi;
      probes[i].lastHeard = now;
      probes[i].isValid   = true;
      return;
    }
  }

  // 3. Table full: Evict oldest entry (LRU)
  size_t oldestIdx = 0;
  unsigned long oldestTime = now;
  
  for (size_t i = 0; i < MAX_PROBES; i++) {
    if (probes[i].lastHeard < oldestTime) {
      oldestTime = probes[i].lastHeard;
      oldestIdx  = i;
    }
  }
  
  // Overwrite oldest
  probes[oldestIdx].id        = id;
  probes[oldestIdx].rssi      = rssi;
  probes[oldestIdx].lastHeard = now;
  probes[oldestIdx].isValid   = true;
}

// =============================================================
//  SEND HELPERS (with radioBusy + CRC)
// =============================================================

bool startTx() {
  if (radioBusy) {
    Serial.println(F("[TX] Busy, dropping"));
    return false;
  }
  radioBusy = true;
  return true;
}

void endTx() {
  radioBusy = false;
}

void sendBeacon() {
  if (!startTx()) return;

  BeaconPacket pkt;
  pkt.type  = PKT_BEACON;
  pkt.srcId = NODE_ID;

  // FIXED: Always advertise our computed rank
  // Children need to see our rank to choose us as parent
  // The JOIN handshake will confirm the relationship
  pkt.rank = (uint8_t)myRank;

  pkt.seq = ++beaconSeq;
  pkt.crc = crc8((uint8_t*)&pkt, sizeof(pkt) - 1);

  int16_t st = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
  if (st == RADIOLIB_ERR_NONE) {
    stats.beaconsSent++;
    Serial.print(F("[TX] BEACON rank="));
    Serial.println(pkt.rank);
  } else {
    Serial.print(F("[TX ERR] BEACON "));
    Serial.println(st);
    stats.txErrors++;
  }
  endTx();
}

void sendJoinRequest() {
  if (bestNextHop == -1 || bestNextHop == SINK_ID) return;
  if (!startTx()) return;

  JoinReqPacket pkt;
  pkt.type     = PKT_JOIN_REQ;
  pkt.srcId    = NODE_ID;
  pkt.parentId = (uint8_t)bestNextHop;
  pkt.seq      = ++dataSeq;
  pkt.crc      = crc8((uint8_t*)&pkt, sizeof(pkt) - 1);

  int16_t st = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
  if (st == RADIOLIB_ERR_NONE) {
    joinRequestSent = millis();
    awaitingJoinAck = true;
    Serial.print(F("[TX] JOIN_REQ -> "));
    Serial.print(bestNextHop);
    Serial.print(F(" seq="));
    Serial.println(pkt.seq);
  } else {
    Serial.print(F("[TX ERR] JOIN_REQ "));
    Serial.println(st);
    stats.txErrors++;
  }
  endTx();
}

// JOIN_ACK includes seq and is used with retries
void sendJoinAck(uint8_t childId, uint16_t seq) {
  if (!startTx()) return;

  JoinAckPacket pkt;
  pkt.type    = PKT_JOIN_ACK;
  pkt.srcId   = NODE_ID;
  pkt.childId = childId;
  pkt.seq     = seq;
  pkt.crc     = crc8((uint8_t*)&pkt, sizeof(pkt) - 1);

  int16_t st = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
  if (st == RADIOLIB_ERR_NONE) {
    Serial.print(F("[TX] JOIN_ACK -> "));
    Serial.print(childId);
    Serial.print(F(" seq="));
    Serial.println(seq);
  } else {
    Serial.print(F("[TX ERR] JOIN_ACK "));
    Serial.println(st);
    stats.txErrors++;
  }
  endTx();
}

void sendAck(uint8_t dstId, uint16_t ackSeq) {
  if (!startTx()) return;

  AckPacket pkt;
  pkt.type   = PKT_ACK;
  pkt.srcId  = NODE_ID;
  pkt.dstId  = dstId;
  pkt.ackSeq = ackSeq;
  pkt.crc    = crc8((uint8_t*)&pkt, sizeof(pkt) - 1);

  int16_t st = radio.transmit((uint8_t*)&pkt, sizeof(pkt));
  if (st == RADIOLIB_ERR_NONE) {
    Serial.print(F("[TX] ACK to "));
    Serial.print(dstId);
    Serial.print(F(" seq="));
    Serial.println(ackSeq);
  } else {
    Serial.print(F("[TX ERR] ACK "));
    Serial.println(st);
    stats.txErrors++;
  }
  endTx();
}

void sendForwardData() {
  if (!forwardState.active || bestNextHop == -1 || !parentConfirmed) return;
  if (!startTx()) return;

  DataPacket pkt;
  pkt.type     = PKT_DATA;
  pkt.srcId    = NODE_ID;
  pkt.dstId    = SINK_ID;  // forward toward sink
  pkt.ttl      = forwardState.ttl;
  pkt.origSrc  = forwardState.origSrc;
  pkt.origSeq  = forwardState.origSeq;
  pkt.seq      = ++dataSeq;
  pkt.payloadLen = forwardState.payloadLen;
  memcpy(pkt.payload, forwardState.payload, pkt.payloadLen);

  const size_t headerSize = offsetof(DataPacket, payload);
  uint8_t calculatedCRC = crc8((uint8_t*)&pkt, headerSize + pkt.payloadLen);

  // Write CRC to the position immediately following the payload
  pkt.payload[pkt.payloadLen] = calculatedCRC;

  size_t packetSize = headerSize + pkt.payloadLen + 1;  // + crc
  int16_t st        = radio.transmit((uint8_t*)&pkt, packetSize);
  if (st == RADIOLIB_ERR_NONE) {
    forwardState.lastSend = millis();
    stats.dataForwarded++;
    Serial.print(F("[TX] DATA -> "));
    Serial.print(bestNextHop);
    Serial.print(F(" ttl="));
    Serial.println(forwardState.ttl);
  } else {
    Serial.print(F("[TX ERR] DATA "));
    Serial.println(st);
    stats.txErrors++;
  }
  endTx();
}

// =============================================================
//  HANDLERS (RX)
// =============================================================

void handleBeaconPacket(const BeaconPacket& pkt, int rssi, unsigned long now) {
  // Always update neighbor info
  updateNeighbor(pkt.srcId, pkt.rank, rssi, now);

  if (pkt.srcId == SINK_ID && pkt.rank == 0) {
    checkDirectSink(rssi);
  }

  // Only compute rank if NOT waiting for JOIN_ACK
  if ((mode == MODE_SEARCHING || mode == MODE_STEADY) &&
      !parentConfirmed && !awaitingJoinAck) {
    computeRank();
  }

  // JOIN-ASSIST LOGIC (parent side)
  if (parentConfirmed && pkt.srcId != SINK_ID) {

    // Start assisting unjoined neighbor
    if (pkt.rank == 255) {
      if (pendingChildId == 0 && pendingJoinSeq == 0) {
        pendingChildId   = pkt.srcId;
        pendingJoinSeq   = 0;
        joinAssistStart  = now;
        joinAckRetries   = 0;
        lastJoinAckSent  = 0;

        Serial.print(F("[JOIN-ASSIST] Saw unjoined neighbor "));
        Serial.print(pendingChildId);
        Serial.println(F(" (rank=255), entering JOIN_HANDLING"));

        transitionTo(MODE_JOIN_HANDLING);
      }
    }

    // Exit assist mode when child joins successfully
    if (mode == MODE_JOIN_HANDLING &&
        pendingChildId == pkt.srcId &&
        pendingJoinSeq == 0 &&
        pkt.rank < 255) {

      Serial.print(F("[JOIN-ASSIST] Child "));
      Serial.print(pendingChildId);
      Serial.println(F(" has joined (rank < 255), back to STEADY"));

      pendingChildId  = 0;
      pendingJoinSeq  = 0;
      joinAckRetries  = 0;
      lastJoinAckSent = 0;
      joinAssistStart = 0;

      transitionTo(MODE_STEADY);
      lastHousekeeping = now;
    }
  }
}

// Parent ignores stale JOIN_REQ from already-joined children
void handleJoinReqPacket(const JoinReqPacket& pkt) {
  if (pkt.parentId != NODE_ID || !parentConfirmed) {
    return;
  }

  Neighbor* child = findNeighbor(pkt.srcId);
  if (child && child->isValid && child->rank < 255) {
    Serial.print(F("[JOIN] Ignoring stale JOIN_REQ from already-joined node "));
    Serial.print(pkt.srcId);
    Serial.print(F(" (rank="));
    Serial.print(child->rank);
    Serial.println(F(")"));
    return;
  }

  Serial.print(F("[JOIN] Request from "));
  Serial.print(pkt.srcId);
  Serial.print(F(" seq="));
  Serial.println(pkt.seq);

  pendingChildId   = pkt.srcId;
  pendingJoinSeq   = pkt.seq;
  joinAckRetries   = 0;
  lastJoinAckSent  = 0;
  joinAssistStart  = millis();

  transitionTo(MODE_JOIN_HANDLING);
}

// Child only confirms parent after successful JOIN_ACK
void handleJoinAckPacket(const JoinAckPacket& pkt) {
  if (!awaitingJoinAck || pkt.childId != NODE_ID) {
    return;
  }

  Serial.print(F("[JOIN] JOIN_ACK received from "));
  Serial.print(pkt.srcId);
  Serial.print(F(" seq="));
  Serial.println(pkt.seq);

  parentConfirmed = true;
  awaitingJoinAck = false;

  // Set rank based on confirmed parent
  Neighbor* parent = findNeighbor(pkt.srcId);
  if (parent && parent->isValid) {
    myRank      = parent->rank + 1;
    bestNextHop = pkt.srcId;

    Serial.print(F("[RANK] Confirmed rank="));
    Serial.print(myRank);
    Serial.print(F(" via parent "));
    Serial.println(bestNextHop);
  }

  // Send final ACK so parent can exit JOIN_HANDLING
  sendAck(pkt.srcId, pkt.seq);

  transitionTo(MODE_STEADY);
}

void handleAckPacket(const AckPacket& pkt, int rssi) {
  (void)rssi;

  // JOIN handshake completion on parent
  if (mode == MODE_JOIN_HANDLING &&
      pendingChildId != 0 &&
      pkt.srcId == pendingChildId &&
      pkt.dstId == NODE_ID &&
      pkt.ackSeq == pendingJoinSeq) {

    Serial.print(F("[JOIN] Child "));
    Serial.print(pendingChildId);
    Serial.print(F(" confirmed JOIN_ACK seq="));
    Serial.println(pendingJoinSeq);

    
    stats.successfulJoins++; // Increment success counter

    pendingChildId  = 0;
    pendingJoinSeq  = 0;
    joinAckRetries  = 0;
    lastJoinAckSent = 0;
    joinAssistStart = 0;
    stats.acksReceived++;

    transitionTo(MODE_STEADY);
    lastHousekeeping = millis();
    return;
  }

  // 1. If ACK is for ME (this repeater) in normal forwarding
  if (pkt.dstId == NODE_ID) {
    if (forwardState.active && pkt.ackSeq == forwardState.origSeq) {
      Serial.println(F("[ACK] Forward confirmed by parent"));
      forwardState.active = false;
      stats.acksReceived++;
    }
    return;
  }

  // 2. If ACK belongs to a child, forward it downward
  Neighbor* child = findNeighbor(pkt.dstId);
  if (child && child->isValid && child->rank > myRank) {
    Serial.print(F("[ACK] Forwarding ACK downward to "));
    Serial.println(pkt.dstId);
    sendAck(pkt.dstId, pkt.ackSeq);
    return;
  }

  // 3. Otherwise ignore
  Serial.println(F("[ACK] Ignored (not for me or my children)"));
}

void handleDataPacket(const DataPacket& pkt, unsigned long now) {
  if (isDuplicate(pkt.origSrc, pkt.origSeq)) {
    stats.duplicatesDropped++;
    Serial.println(F("[DATA] Duplicate dropped"));
    return;
  }
  markSeen(pkt.origSrc, pkt.origSeq);

  if (pkt.ttl == 0) {
    Serial.println(F("[DATA] TTL expired"));
    return;
  }

  bool shouldForwardUp =
    (pkt.dstId == NODE_ID) || (pkt.dstId == SINK_ID) || (pkt.dstId == DST_BROADCAST);
  if (shouldForwardUp) {

    // Local hop ACK — parent acknowledges child immediately
    sendAck(pkt.srcId, pkt.origSeq);
    Serial.print(F("[ACK] Local ACK sent to child "));
    Serial.println(pkt.srcId);

    // inspect and print sensor payload
    if (pkt.payloadLen == sizeof(SensorPayload)) {
      SensorPayload sp;
      // Safe copy to struct
      memcpy(&sp, pkt.payload, sizeof(SensorPayload));
      
      Serial.print(F("[SENSOR] ID="));
      Serial.print(sp.nodeId);
      Serial.print(F(" Air="));
      Serial.print(sp.tempAir / 100.0f);
      Serial.print(F("C Soil="));
      Serial.print(sp.tempSoil / 100.0f);
      Serial.print(F("C Hum="));
      Serial.print(sp.humidity / 100.0f);
      Serial.print(F("% Lux="));
      Serial.print(sp.lux);
      Serial.print(F(" Moist="));
      Serial.println(sp.moisture);
    }

    // Queue packet for forwarding up the tree
    if (parentConfirmed && bestNextHop != -1 && !forwardState.active) {
      forwardState.active    = true;
      forwardState.origSrc   = pkt.origSrc;
      forwardState.origSeq   = pkt.origSeq;
      forwardState.ttl       = pkt.ttl - 1;
      forwardState.payloadLen = min(pkt.payloadLen, (uint8_t)MAX_PAYLOAD_LEN);
      memcpy(forwardState.payload, pkt.payload, forwardState.payloadLen);
      forwardState.retries   = 0;
      forwardState.lastSend  = 0;

      Serial.println(F("[DATA] Queued for forwarding"));
    } else {
      Serial.println(F("[DATA] Cannot forward (no parent or busy)"));
    }
  }
}

void handleProbeReqPacket(const ProbeReqPacket& pkt, int rssi, unsigned long now) {
  stats.probeReqsReceived++;
  updateProbes(pkt.srcId, rssi, now);

  (void)rssi;
  (void)now;

  // FIXED: Answer if we have ANY rank, even if not yet confirmed
  // This allows sensors to discover us during bootstrap
  if (myRank >= 255) {
    return;  // Only skip if we have NO rank at all
  }

  if (!startTx()) return;

  ProbeRespPacket resp;
  resp.type  = PKT_PROBE_RESP;
  resp.srcId = NODE_ID;
  resp.rank  = (uint8_t)myRank;
  resp.seq   = pkt.seq;
  resp.crc   = crc8((uint8_t*)&resp, sizeof(resp) - 1);

  int16_t st = radio.transmit((uint8_t*)&resp, sizeof(resp));
  if (st == RADIOLIB_ERR_NONE) {
    Serial.print(F("[PROBE] RESP -> sensor "));
    Serial.print(pkt.srcId);
    Serial.print(F(" rank="));
    Serial.print(myRank);
    Serial.print(F(" seq="));
    Serial.println(pkt.seq);
  } else {
    Serial.print(F("[PROBE][ERR] RESP TX "));
    Serial.println(st);
    stats.txErrors++;
  }

  endTx();
}

// =============================================================
//  RADIO POLL (with CRC & length checks)
// =============================================================

void pollRadio() {
  if (radioBusy) {
    Serial.println("[RADIO] Busy flag stuck! Rx skipped!");
    return;
  }

  int16_t st = radio.receive(rxBuf, RX_BUFFER_SIZE);
  if (st == RADIOLIB_ERR_NONE) {
    size_t len = radio.getPacketLength();
    if (len == 0 || len > RX_BUFFER_SIZE) {
      stats.malformedPackets++;
      return;
    }

    int           rssi = (int)radio.getRSSI();
    unsigned long now  = millis();

    uint8_t type = rxBuf[0];
    switch (type) {
      case PKT_BEACON:
        {
          if (len < sizeof(BeaconPacket)) {
            stats.malformedPackets++;
            Serial.println(F("[RX] Beacon too short"));
            break;
          }
          BeaconPacket bp;
          memcpy(&bp, rxBuf, sizeof(BeaconPacket));
          uint8_t calc = crc8((uint8_t*)&bp, sizeof(bp) - 1);
          if (calc != bp.crc) {
            stats.crcErrors++;
            Serial.println(F("[CRC] Beacon CRC error"));
            break;
          }
          handleBeaconPacket(bp, rssi, now);
          break;
        }

      case PKT_DATA:
        {
          size_t headerSize = offsetof(DataPacket, payload);
          if (len < headerSize + 1) {
            stats.malformedPackets++;
            Serial.println(F("[RX] Data too short (header)"));
            break;
          }

          DataPacket dp;
          memset(&dp, 0, sizeof(dp));
          memcpy(&dp, rxBuf, headerSize);  // includes payloadLen
          if (dp.payloadLen > MAX_PAYLOAD_LEN) {
            stats.malformedPackets++;
            Serial.println(F("[RX] Data payload too large"));
            break;
          }

          size_t needed = headerSize + dp.payloadLen + 1;  // + crc
          if (len < needed) {
            stats.malformedPackets++;
            Serial.println(F("[RX] Data length mismatch"));
            break;
          }

          memcpy(dp.payload, rxBuf + headerSize, dp.payloadLen);
          dp.crc = rxBuf[headerSize + dp.payloadLen];

          uint8_t calc = crc8((uint8_t*)&dp, headerSize + dp.payloadLen);
          if (calc != dp.crc) {
            stats.crcErrors++;
            Serial.println(F("[CRC] Data CRC error"));
            break;
          }

          handleDataPacket(dp, now);
          break;
        }

      case PKT_ACK:
        {
          if (len < sizeof(AckPacket)) {
            stats.malformedPackets++;
            Serial.println(F("[RX] Ack too short"));
            break;
          }
          AckPacket ap;
          memcpy(&ap, rxBuf, sizeof(AckPacket));
          uint8_t calc = crc8((uint8_t*)&ap, sizeof(ap) - 1);
          if (calc != ap.crc) {
            stats.crcErrors++;
            Serial.println(F("[CRC] Ack CRC error"));
            break;
          }
          int rssi2 = radio.getRSSI();
          handleAckPacket(ap, rssi2);
          break;
        }

      case PKT_JOIN_REQ:
        {
          if (len < sizeof(JoinReqPacket)) {
            stats.malformedPackets++;
            Serial.println(F("[RX] JoinReq too short"));
            break;
          }
          JoinReqPacket jp;
          memcpy(&jp, rxBuf, sizeof(JoinReqPacket));
          uint8_t calc = crc8((uint8_t*)&jp, sizeof(jp) - 1);
          if (calc != jp.crc) {
            stats.crcErrors++;
            Serial.println(F("[CRC] JoinReq CRC error"));
            break;
          }
          handleJoinReqPacket(jp);
          break;
        }

      case PKT_JOIN_ACK:
        {
          if (len < sizeof(JoinAckPacket)) {
            stats.malformedPackets++;
            Serial.println(F("[RX] JoinAck too short"));
            break;
          }
          JoinAckPacket ja;
          memcpy(&ja, rxBuf, sizeof(JoinAckPacket));
          uint8_t calc = crc8((uint8_t*)&ja, sizeof(ja) - 1);
          if (calc != ja.crc) {
            stats.crcErrors++;
            Serial.println(F("[CRC] JoinAck CRC error"));
            break;
          }
          handleJoinAckPacket(ja);
          break;
        }

      case PKT_PROBE_REQ:
        {
          if (len < sizeof(ProbeReqPacket)) {
            stats.malformedPackets++;
            Serial.println(F("[RX] ProbeReq too short"));
            break;
          }
          ProbeReqPacket pr;
          memcpy(&pr, rxBuf, sizeof(ProbeReqPacket));
          uint8_t calc = crc8((uint8_t*)&pr, sizeof(pr) - 1);
          if (calc != pr.crc) {
            stats.crcErrors++;
            Serial.println(F("[CRC] ProbeReq CRC error"));
            break;
          }
          handleProbeReqPacket(pr, rssi, now);
          break;
        }

      case PKT_PROBE_RESP:
        // Repeater ignores probe responses (these are for sensors)
        break;

      default:
        Serial.print(F("[RX] Unknown type "));
        Serial.println(type);
        stats.malformedPackets++;
        break;
    }

  } else if (st != RADIOLIB_ERR_RX_TIMEOUT) {
    Serial.print(F("[RX ERR] "));
    Serial.println(st);
    stats.rxErrors++;
  }
}

// =============================================================
//  BEACONS / FORWARD QUEUE / SANITY
// =============================================================

void scheduleNextBeacon(unsigned long now) {
  unsigned long base   = (mode == MODE_STEADY) ? STEADY_BEACON_INTERVAL
                                               : SEARCH_BEACON_INTERVAL;
  unsigned long jitter = (mode == MODE_STEADY) ? 1000UL : SEARCH_BEACON_JITTER;
  nextBeaconAt         = now + base + random(0UL, jitter);
}

void maybeSendBeacon(unsigned long now) {
  if (mode != MODE_SEARCHING &&
      mode != MODE_STEADY &&
      mode != MODE_JOIN_HANDLING) return;

  if (forwardState.active) {
    nextBeaconAt = now + 500UL;
    return;
  }

  if (now >= nextBeaconAt) {
    sendBeacon();
    scheduleNextBeacon(now);
  }
}

void processForwardQueue(unsigned long now) {
  if (!forwardState.active) return;
  if (bestNextHop == -1 || !parentConfirmed) {
    Serial.println(F("[FWD] No parent, dropping"));
    forwardState.active = false;
    return;
  }

  if (forwardState.lastSend == 0 ||
      !isWithin(now, forwardState.lastSend, ACK_TIMEOUT)) {

    if (forwardState.lastSend != 0) {
      forwardState.retries++;
      if (forwardState.retries >= MAX_RETRIES) {
        Serial.println(F("[FWD] Max retries, dropping"));
        forwardState.active = false;
        return;
      }
      Serial.print(F("[FWD] Retry "));
      Serial.println(forwardState.retries);
    }

    sendForwardData();
  }
}

void sanityCheckParent(unsigned long now) {
  if (!parentConfirmed) return;
  // If we are directly attached to sink with rank=1, no need to recompute
  if (bestNextHop == SINK_ID && myRank == 1) return;
  if (mode != MODE_STEADY && mode != MODE_SEARCHING) return;
  if (isWithin(now, lastParentSanity, PARENT_SANITY_INTERVAL)) return;

  lastParentSanity = now;
  Serial.println(F("[SANITY] Checking parent"));
  computeRank();
}

// =============================================================
//  DIAGNOSTICS
// =============================================================

void printStats() {
  Serial.println(F("\n=== STATS ==="));
  Serial.print(F("Mode: "));
  Serial.println(mode);
  Serial.print(F("Rank: "));
  Serial.println(myRank);
  Serial.print(F("Parent: "));
  Serial.println(bestNextHop);
  Serial.print(F("Confirmed: "));
  Serial.println(parentConfirmed);
  Serial.print(F("Beacons: "));
  Serial.println(stats.beaconsSent);
  Serial.print(F("Fwd: "));
  Serial.println(stats.dataForwarded);
  Serial.print(F("ACKs: "));
  Serial.println(stats.acksReceived);
  Serial.print(F("Dup: "));
  Serial.println(stats.duplicatesDropped);
  Serial.print(F("Parent switches: "));
  Serial.println(stats.parentSwitches);
  Serial.print(F("TX errors: "));
  Serial.println(stats.txErrors);
  Serial.print(F("RX errors: "));
  Serial.println(stats.rxErrors);
  Serial.print(F("CRC errors: "));
  Serial.println(stats.crcErrors);
  Serial.print(F("Malformed: "));
  Serial.println(stats.malformedPackets);
  Serial.print(F("Received Probe Requests: "));            
  Serial.println(stats.probeReqsReceived);
  Serial.print(F("Joins Success: "));
  Serial.println(stats.successfulJoins);
  Serial.print(F("Neighbor timeouts: "));
  Serial.println(stats.neighborTimeouts);

  // ---------- NEIGHBOR COUNTER ----------

  Serial.println(F("\nNeighbors:"));
  for (size_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighbors[i].isValid) {
      Serial.print(F("ID "));
      Serial.print(neighbors[i].id);
      Serial.print(F(" rank="));
      Serial.print(neighbors[i].rank);
      Serial.print(F(" rssi="));
      Serial.print(neighbors[i].rssi);
      unsigned long age = (millis() - neighbors[i].lastHeard) / 1000UL;
      Serial.print(F(" age="));
      Serial.print(age);
      Serial.println(F("s"));
    }
  }
  
  // ---------- PROBE COUNTER ----------

  Serial.println(F("\nRecent Probes:"));
    bool foundProbe = false;
    for (size_t i = 0; i < MAX_PROBES; i++) {
      if (probes[i].isValid) {
        foundProbe = true;
        Serial.print(F("ID "));
        Serial.print(probes[i].id);
        Serial.print(F(" rssi="));
        Serial.print(probes[i].rssi);

        unsigned long age = (millis() - probes[i].lastHeard) / 1000UL;
        Serial.print(F(" age="));
        Serial.print(age);
        Serial.println(F("s"));
      }
    }
    if (!foundProbe) {
      Serial.println(F("(none)"));
    }

  Serial.println();
  }

void handleSerialCommand() {
  if (!Serial.available()) return;
  char c = Serial.read();
  while (Serial.available()) Serial.read();
  switch (c) {
    case 's':
    case 'S':
      printStats();
      break;
    case 'r':
    case 'R':
      Serial.println(F("[CMD] Force REPAIR"));
      transitionTo(MODE_REPAIR);
      break;
    case 'h':
    case 'H':
      Serial.println(F("Commands: s=stats, r=repair, h=help"));
      break;
  }
}

// =============================================================
//  SETUP / LOOP
// =============================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  if (NODE_ID == SINK_ID) {
    Serial.println(F("[FATAL] NODE_ID == SINK_ID"));
    while (1) { }
  }

  randomSeed(micros());

  radio.setRfSwitchTable(RFSW_PINS, RFSW_TABLE);
  int16_t st = radio.begin(
    FREQ_MHZ, BW_KHZ, SF, CR, SYNCWORD, PWR_DBM, PREAMBLE, TCXO_V, USE_LDO);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.print(F("[FATAL] radio.begin "));
    Serial.println(st);
    while (1) { }
  }
  radio.setCRC(true);  // PHY-level CRC

  Serial.println(F("\n=== SIMPLE BINARY REPEATER (CRC + JOIN HANDSHAKE + PROBE RESP) ==="));
  Serial.print(F("NODE_ID="));
  Serial.println(NODE_ID);

  mode = MODE_BOOT_LISTEN;
  bootListenStart = millis();
  scheduleNextBeacon(millis());
}

void loop() {
  pollRadio();

  unsigned long now = millis();
  handleSerialCommand();

  switch (mode) {
    case MODE_STEADY:
      pruneNeighbors(now);
      processForwardQueue(now);
      sanityCheckParent(now);
      maybeSendBeacon(now);

      if (now - lastHousekeeping >= HOUSEKEEPING_INTERVAL) {
        lastHousekeeping = now;
        Serial.println(F("[STEADY] 5-minute heartbeat"));
      }
      break;

    case MODE_JOIN_HANDLING:
      {
        // Keep forwarding data while we handle join / assist
        processForwardQueue(now);

        // A) JOIN-ASSIST MODE: pendingChildId != 0, pendingJoinSeq == 0
        if (pendingChildId != 0 && pendingJoinSeq == 0) {

          // If we've been assisting too long, give up
          if (!isWithin(now, joinAssistStart, JOIN_ASSIST_TIMEOUT)) {
            Serial.println(F("[HANDSHAKE] Join-assist timeout, back to STEADY"));
            pendingChildId  = 0;
            joinAssistStart = 0;
            joinAckRetries  = 0;
            lastJoinAckSent = 0;

            transitionTo(MODE_STEADY);
            break;
          }

          // While assisting, maintain network health
          pruneNeighbors(now);
          sanityCheckParent(now);
          maybeSendBeacon(now);  // keep advertising our good rank

          // We do NOT send JOIN_ACK here; we still need a JOIN_REQ with seq.
          break;
        }

        // B) NORMAL JOIN HANDSHAKE MODE: pendingChildId != 0, pendingJoinSeq != 0
        if (pendingChildId == 0 || pendingJoinSeq == 0) {
          Serial.println(F("[HANDSHAKE] No pending child, back to STEADY"));
          transitionTo(MODE_STEADY);
          break;
        }

        if (joinAckRetries >= JOIN_ACK_MAX_RETRIES) {
          Serial.println(F("[HANDSHAKE] Max JOIN_ACK retries, back to STEADY"));
          pendingChildId  = 0;
          pendingJoinSeq  = 0;
          joinAckRetries  = 0;
          lastJoinAckSent = 0;
          joinAssistStart = 0;
          transitionTo(MODE_STEADY);
          break;
        }

        if (lastJoinAckSent == 0 ||
            !isWithin(now, lastJoinAckSent, JOIN_ACK_RETRY_INTERVAL)) {

          Serial.print(F("[HANDSHAKE] Sending JOIN_ACK to "));
          Serial.print(pendingChildId);
          Serial.print(F(" seq="));
          Serial.print(pendingJoinSeq);
          Serial.print(F(" (try "));
          Serial.print(joinAckRetries + 1);
          Serial.println(F(")"));

          sendJoinAck(pendingChildId, pendingJoinSeq);
          lastJoinAckSent = now;
          joinAckRetries++;
        }
        break;
      }

    case MODE_BOOT_LISTEN:
      pruneNeighbors(now);
      processForwardQueue(now);
      sanityCheckParent(now);
      maybeSendBeacon(now);

      if (!isWithin(now, bootListenStart, BOOT_LISTEN_DURATION)) {
        Serial.println(F("[BOOT] Done, entering SEARCHING"));
        transitionTo(MODE_SEARCHING);
        searchWindowStart = now;
        scheduleNextBeacon(now);
      }
      break;

    case MODE_SEARCHING:
      pruneNeighbors(now);
      processForwardQueue(now);
      sanityCheckParent(now);
      maybeSendBeacon(now);

      // 1) Already sent JOIN_REQ → wait for JOIN_ACK or timeout
      if (awaitingJoinAck) {
        if (!isWithin(now, joinRequestSent, JOIN_REQ_TIMEOUT)) {
          Serial.println(F("[JOIN] Timeout, retry search"));
          awaitingJoinAck   = false;
          searchWindowStart = now;  // restart search window
        }
        break;
      }

      // 2) Not waiting, not confirmed, but we already have a parent:
      //    -> send JOIN_REQ immediately (or go STEADY if direct sink)
      if (!parentConfirmed && bestNextHop != -1) {
        if (bestNextHop == SINK_ID) {
          parentConfirmed = true;
          resetParentCandidate();
          transitionTo(MODE_STEADY);
          lastHousekeeping = now;
        } else {
          sendJoinRequest();
        }
        break;
      }

      // 3) No parent yet: periodically recompute rank to see new neighbors
      if (!parentConfirmed && bestNextHop == -1 &&
          !isWithin(now, searchWindowStart, SEARCH_WINDOW_MS)) {
        computeRank();
        searchWindowStart = now;
      }
      break;

    case MODE_REPAIR:
      parentConfirmed       = false;
      myRank                = 255;
      bestNextHop           = -1;
      awaitingJoinAck       = false;
      forwardState.active   = false;
      resetParentCandidate();
      transitionTo(MODE_SEARCHING);
      searchWindowStart = now;
      scheduleNextBeacon(now);
      break;
  }
}
