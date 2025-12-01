#include <Arduino.h>
#include <RadioLib.h>

STM32WLx_Module wl;
STM32WLx radio(&wl);

// RF switch table for Wio-E5 (PA4/PA5 required)
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

static const float FREQ_MHZ   = 923.2f;
static const float BW_KHZ     = 500.0f;
static const uint8_t SF       = 5;
static const uint8_t CR       = 5;
static const uint8_t SYNCWORD = 0x12;
static const int8_t PWR_DBM   = -5;
static const uint16_t PREAMBLE = 8;
static const float TCXO_V     = 1.6f;
static const bool USE_LDO     = false;

static const int NODE_ID = 1;  // this repeater
static const int SINK_ID = 0;  // final receiver / Pi

static const unsigned long BEACON_INTERVAL       = 5000UL;
static const unsigned long BEACON_JITTER         = 2000UL;
static const unsigned long NEIGHBOR_TIMEOUT      = 30000UL;
static const int           RSSI_THRESHOLD        = -115;
static const unsigned long RANK_SETTLE_TIME      = 60000UL;
static const uint8_t       MAX_HOPS              = 5;
static const unsigned long ROUTE_UPDATE_INTERVAL = 10000UL;
static const unsigned long DEDUP_TIMEOUT         = 30000UL;
static const unsigned long ACK_TIMEOUT           = 3000UL;
static const uint8_t       MAX_RETRIES           = 3;

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

struct Neighbor {
  int id = -1;
  int rank = 255;
  int rssi = -200;
  unsigned long lastHeard = 0;
  bool isValid = false;
};

struct SeenPacket {
  int originatorId = -1;
  int sequenceNumber = -1;
  unsigned long timestamp = 0;
  bool valid = false;
};

struct ForwardState {
  bool active = false;
  int sensorId = -1;
  uint16_t seq = 0;
  int ttl = 0;
  String payload;
  uint8_t retries = 0;
  unsigned long lastSend = 0;
};

static const size_t MAX_NEIGHBORS = 15;
static const size_t CACHE_SIZE    = 20;

Neighbor   neighborTable[MAX_NEIGHBORS];
SeenPacket seenCache[CACHE_SIZE];

uint16_t beaconSeq  = 0;
uint16_t controlSeq = 0;

int myRank          = 255;
int bestNextHop     = -1;
unsigned long lastRankUpdate = 0;
unsigned long lastBeacon     = 0;
bool needsRankUpdate         = false;

ForwardState forwardState;

uint16_t nextBeaconSeq() {
  return ++beaconSeq;
}

uint16_t nextControlSeq() {
  return ++controlSeq;
}

String buildMessage(const char* type, int src, int dst, int seq,
                    int rank = -1, int ttl = -1, int orig = -1,
                    int ack = -1, const String& data = "") {
  String msg = "TYPE=";
  msg += type;
  msg += ";SRC=";
  msg += src;
  msg += ";DST=";
  msg += dst;
  msg += ";SEQ=";
  msg += seq;

  if (rank >= 0) {
    msg += ";RANK=";
    msg += rank;
  }

  if (ttl >= 0) {
    msg += ";TTL=";
    msg += ttl;
  }

  if (orig >= 0) {
    msg += ";ORIG=";
    msg += orig;
  }

  if (ack >= 0) {
    msg += ";ACK=";
    msg += ack;
  }

  if (data.length() > 0) {
    msg += ";DATA=";
    msg += data;
  }

  return msg;
}

String buildBeaconMessage() {
  return buildMessage("BEACON", NODE_ID, -1, nextBeaconSeq(), myRank);
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
    if (end < 0) end = header.length();

    int eq = header.indexOf('=', start);
    if (eq > start && eq < end) {
      String key = header.substring(start, eq);
      String val = header.substring(eq + 1, end);

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

void sendMessage(String& payload) {
  delay(random(10, 50));
  int16_t state = radio.transmit(payload);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.print(F("[TX] "));
    Serial.println(payload);
  } else {
    Serial.print(F("[TX][ERR] "));
    Serial.println(state);
  }
}

void sendBeacon() {
  String msg = buildBeaconMessage();
  sendMessage(msg);
  lastBeacon = millis();
  Serial.print(F("[BEACON] rank="));
  Serial.println(myRank);
}

Neighbor* findNeighbor(int nodeId) {
  for (size_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighborTable[i].isValid && neighborTable[i].id == nodeId) {
      return &neighborTable[i];
    }
  }
  return nullptr;
}

Neighbor* findEmptySlot() {
  for (size_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (!neighborTable[i].isValid) {
      return &neighborTable[i];
    }
  }
  return nullptr;
}

void clearNeighborTable() {
  for (size_t i = 0; i < MAX_NEIGHBORS; i++) {
    neighborTable[i].isValid = false;
  }
}

void updateNeighbor(int nodeId, int rank, int rssi, unsigned long now) {
  Neighbor* slot = findNeighbor(nodeId);

  if (slot == nullptr) {
    slot = findEmptySlot();
  }

  if (slot == nullptr) {
    Serial.print(F("[NB] Table full, ignoring "));
    Serial.println(nodeId);
    return;
  }

  slot->id = nodeId;
  slot->rank = rank;
  slot->rssi = rssi;
  slot->lastHeard = now;
  slot->isValid = true;

  Serial.print(F("[NB] Updated neighbor "));
  Serial.print(nodeId);
  Serial.print(F(" rank="));
  Serial.print(rank);
  Serial.print(F(" rssi="));
  Serial.println(rssi);
}

void pruneStaleNeighbors(unsigned long now) {
  for (size_t i = 0; i < MAX_NEIGHBORS; i++) {
    if (neighborTable[i].isValid) {
      if ((now - neighborTable[i].lastHeard) > NEIGHBOR_TIMEOUT) {
        Serial.print(F("[NB] Neighbor timeout: "));
        Serial.println(neighborTable[i].id);
        if (neighborTable[i].id == bestNextHop) {
          bestNextHop = -1;
          needsRankUpdate = true;
        }
        neighborTable[i].isValid = false;
      }
    }
  }
}

int findBestNextHop() {
  Neighbor* bestNeighbor = nullptr;
  int bestRssi = -999;

  for (size_t i = 0; i < MAX_NEIGHBORS; i++) {
    Neighbor& nb = neighborTable[i];
    if (!nb.isValid) continue;
    if (nb.rank >= myRank) continue;
    if (nb.rssi < RSSI_THRESHOLD) continue;

    if (nb.rssi > bestRssi) {
      bestRssi = nb.rssi;
      bestNeighbor = &nb;
    }
  }

  if (bestNeighbor != nullptr) {
    return bestNeighbor->id;
  }
  return -1;
}

void updateMyRank() {
  int oldRank = myRank;
  int minNeighborRank = 255;

  for (size_t i = 0; i < MAX_NEIGHBORS; i++) {
    Neighbor& nb = neighborTable[i];
    if (nb.isValid && nb.rssi >= RSSI_THRESHOLD && nb.rank < minNeighborRank) {
      minNeighborRank = nb.rank;
    }
  }

  if (minNeighborRank < 255) {
    myRank = minNeighborRank + 1;
    if (myRank > MAX_HOPS) {
      myRank = 255;
    }
  } else {
    myRank = 255;
  }

  if (myRank != oldRank) {
    Serial.print(F("[RANK] Changed: "));
    Serial.print(oldRank);
    Serial.print(F(" -> "));
    Serial.println(myRank);
    lastRankUpdate = millis();
  }
}

void updateRoutingState(unsigned long now) {
  // Only recompute if enough time has passed OR we were explicitly asked to
  if ((now - lastRankUpdate) > ROUTE_UPDATE_INTERVAL || needsRankUpdate) {
    updateMyRank();
    bestNextHop = findBestNextHop();

    if (bestNextHop != -1) {
      Serial.print(F("[ROUTE] Best next hop: "));
      Serial.println(bestNextHop);
    } else {
      Serial.println(F("[ROUTE] WARNING: No route to sink"));
    }

    needsRankUpdate = false;
    lastRankUpdate = now;
  }
}

SeenPacket* findOldestCacheSlot() {
  size_t oldestIdx = 0;
  unsigned long oldestTime = UINT32_MAX;

  for (size_t i = 0; i < CACHE_SIZE; i++) {
    if (!seenCache[i].valid) {
      return &seenCache[i];
    }
    if (seenCache[i].timestamp < oldestTime) {
      oldestTime = seenCache[i].timestamp;
      oldestIdx = i;
    }
  }

  return &seenCache[oldestIdx];
}

bool hasSeenPacket(int origId, int seqNum) {
  unsigned long now = millis();

  for (size_t i = 0; i < CACHE_SIZE; i++) {
    if (seenCache[i].valid && (now - seenCache[i].timestamp) > DEDUP_TIMEOUT) {
      seenCache[i].valid = false;
    }

    if (seenCache[i].valid &&
        seenCache[i].originatorId   == origId &&
        seenCache[i].sequenceNumber == seqNum) {
      return true;
    }
  }

  return false;
}

void addToSeenCache(int origId, int seqNum) {
  SeenPacket* slot = findOldestCacheSlot();
  slot->originatorId   = origId;
  slot->sequenceNumber = seqNum;
  slot->timestamp      = millis();
  slot->valid          = true;
}

void clearSeenCache() {
  for (size_t i = 0; i < CACHE_SIZE; i++) {
    seenCache[i].valid = false;
  }
}

void forwardToSink() {
  if (!forwardState.active) return;

  if (bestNextHop == -1) {
    Serial.println(F("[FWD] No route to sink, cannot forward"));
    forwardState.active = false;
    return;
  }

  if (forwardState.ttl < 0) {
    Serial.println(F("[FWD] TTL expired, dropping"));
    forwardState.active = false;
    return;
  }

  String packet = buildMessage(
    "DATA",
    NODE_ID,
    bestNextHop,
    forwardState.seq,
    -1,
    forwardState.ttl,
    forwardState.sensorId,
    -1,
    forwardState.payload
  );

  sendMessage(packet);
  forwardState.lastSend = millis();
  forwardState.retries++;

  Serial.print(F("[FWD] Forwarded to "));
  Serial.print(bestNextHop);
  Serial.print(F(" ttl="));
  Serial.println(forwardState.ttl);
}

void acknowledgeSensor() {
  if (!forwardState.active) return;

  String ack = buildMessage(
    "ACK",
    NODE_ID,
    forwardState.sensorId,
    nextControlSeq(),
    -1,
    -1,
    -1,
    forwardState.seq
  );
  sendMessage(ack);

  forwardState.active = false;
  Serial.println(F("[ACK] Forwarding complete, ACKed sensor"));
}

void handleDataFromSensor(const MeshMessage& msg) {
  if (!msg.hasSeq || !msg.hasTtl || !msg.hasData) {
    Serial.println(F("[RX] Malformed DATA from sensor"));
    return;
  }

  if (msg.ttl <= 0) {
    Serial.println(F("[RX] Dropping packet with expired TTL"));
    return;
  }

  if (hasSeenPacket(msg.src, msg.seq)) {
    Serial.println(F("[RX] Duplicate packet, ignoring"));
    return;
  }

  if (forwardState.active) {
    // Retransmission of the same packet → retry forward
    if (msg.seq == forwardState.seq && msg.src == forwardState.sensorId) {
      Serial.println(F("[FWD] Retransmission detected, will retry"));
      forwardState.ttl     = msg.ttl - 1;
      forwardState.payload = msg.data;
      forwardState.retries = 0;
      forwardToSink();
    } else {
      Serial.println(F("[FWD] Busy forwarding, rejecting new packet"));
    }
    return;
  }

  Serial.print(F("[FWD] Received DATA from sensor "));
  Serial.print(msg.src);
  Serial.print(F(" seq="));
  Serial.println(msg.seq);

  addToSeenCache(msg.src, msg.seq);

  forwardState.active   = true;
  forwardState.sensorId = msg.src;
  forwardState.seq      = msg.seq;
  forwardState.ttl      = msg.ttl - 1;
  forwardState.payload  = msg.data;
  forwardState.retries  = 0;

  forwardToSink();
}

void handleAck(const MeshMessage& msg) {
  if (!forwardState.active || !msg.hasAck) return;
  if (msg.ack != forwardState.seq) return;

  Serial.print(F("[ACK] Received for seq "));
  Serial.println(msg.ack);

  acknowledgeSensor();
}

void handleBeacon(const MeshMessage& msg, int rssi, unsigned long now) {
  if (!msg.hasRank) return;

  updateNeighbor(msg.src, msg.rank, rssi, now);

  // If we hear the sink directly (ID 0, rank 0), we know our rank = 1
  if (msg.src == SINK_ID && msg.rank == 0) {
    int oldRank = myRank;
    myRank = 1;
    bestNextHop = SINK_ID;
    Serial.print(F("[RANK] Direct link to sink, "));
    Serial.print(oldRank);
    Serial.print(F(" -> "));
    Serial.println(myRank);
  }

  needsRankUpdate = true;

  Serial.print(F("[BEACON][RX] from "));
  Serial.print(msg.src);
  Serial.print(F(" rank="));
  Serial.print(msg.rank);
  Serial.print(F(" rssi="));
  Serial.println(rssi);
}

void processIncoming(const MeshMessage& msg, int rssi, unsigned long now) {
  if (!msg.valid) return;

  if (msg.type == "BEACON") {
    handleBeacon(msg, rssi, now);
  } else if (msg.type == "DATA" && msg.dst == NODE_ID) {
    handleDataFromSensor(msg);
  } else if (msg.type == "ACK" && msg.dst == NODE_ID) {
    handleAck(msg);
  }
}

void pollRadio() {
  for (int i = 0; i < 4; i++) {
    String incoming;
    int16_t st = radio.receive(incoming);  // no rssi param here

    if (st == RADIOLIB_ERR_NONE) {
      float rssiF = radio.getRSSI();       // ask RadioLib after RX
      int16_t rssi = (int16_t)rssiF;

      Serial.print(F("[RX] "));
      Serial.print(incoming);
      Serial.print(F(" rssi="));
      Serial.println(rssi);

      MeshMessage msg = parseMessage(incoming);
      processIncoming(msg, rssi, millis());

    } else if (st == RADIOLIB_ERR_RX_TIMEOUT || st == RADIOLIB_ERR_CRC_MISMATCH) {
      break;
    } else {
      Serial.print(F("[RX][ERR] "));
      Serial.println(st);
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  randomSeed(analogRead(0));

  Serial.println();
  Serial.println(F("=== Mesh Repeater (Dynamic Rank) ==="));

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
    Serial.println(F("[FATAL] LoRa init failed!"));
    while (true) {
      delay(1000);
    }
  }

  radio.setCRC(true);
  clearNeighborTable();
  clearSeenCache();

  myRank         = 255;
  bestNextHop    = -1;
  lastRankUpdate = millis();
  lastBeacon     = millis();

  Serial.println(F("[INIT] Repeater ready, discovering network..."));
}

void loop() {
  unsigned long now = millis();

  unsigned long nextBeaconDelay = BEACON_INTERVAL + random(0, BEACON_JITTER);
  if ((now - lastBeacon) >= nextBeaconDelay) {
    sendBeacon();
  }

  // RX can update neighbor timestamps using a *newer* millis()
  pollRadio();

  // Refresh 'now' AFTER RX so it's >= any lastHeard we just wrote
  now = millis();

  pruneStaleNeighbors(now);
  updateRoutingState(now);

  if (forwardState.active) {
    if ((now - forwardState.lastSend) > ACK_TIMEOUT) {
      if (forwardState.retries < MAX_RETRIES) {
        Serial.println(F("[FWD] No ACK yet, retrying"));
        forwardToSink();
      } else {
        Serial.println(F("[FWD] Retries exhausted, dropping packet"));
        forwardState.active = false;
      }
    }
  }

  delay(50);
}