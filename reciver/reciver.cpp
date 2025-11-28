#include <RadioLib.h>
#include "hal/RPi/PiHal.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <chrono>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>

// ================== Radio / Mesh Config ==================

static PiHal* hal = new PiHal(0);                  // SPI channel 0 (CE0)
// NSS=8, DIO1=25, NRST=23, BUSY=24
static SX1262 radio = new Module(hal, 8, 25, 23, 24);

static const int SINK_ID = 0;                      // This node (base)

// Keep LoRa settings aligned with sensor/repeater
static const float   LORA_FREQ_MHZ   = 923.2f;
static const float   LORA_BW_KHZ     = 500.0f;
static const uint8_t LORA_SF         = 5;
static const uint8_t LORA_CR         = 5;
static const uint8_t LORA_SYNCWORD   = 0x12;
static const int8_t  LORA_PWR_DBM    = 22;
static const uint16_t LORA_PREAMBLE  = 8;

static const unsigned long BEACON_INTERVAL_MS = 5000UL;
static const unsigned long BEACON_JITTER_MS   = 2000UL;
static const unsigned long RX_TIMEOUT_MS      = 100UL;
static const unsigned long DEDUP_TIMEOUT_MS   = 30000UL;

static uint16_t beaconSeq = 0;
static uint16_t ackSeq = 0;

// CSV logging
static std::ofstream csvFile;
static const char* CSV_FILENAME = "sensor_data.csv";
static bool csvHeaderWritten = false;

// Deduplication cache
struct PacketKey {
    int originatorId;
    int sequenceNumber;
    
    bool operator==(const PacketKey& other) const {
        return originatorId == other.originatorId && 
               sequenceNumber == other.sequenceNumber;
    }
};

struct PacketKeyHash {
    std::size_t operator()(const PacketKey& k) const {
        return std::hash<int>()(k.originatorId) ^ 
               (std::hash<int>()(k.sequenceNumber) << 1);
    }
};

std::unordered_map<PacketKey, std::chrono::steady_clock::time_point, PacketKeyHash> seenPackets;

// ================== Packet Structure =====================

struct MeshPacket {
    bool        valid  = false;
    std::string type;
    int         src    = -1;
    int         dst    = -1;
    int         seq    = -1;
    int         ttl    = -1;
    int         rank   = -1;
    int         orig   = -1;  // Original sensor ID for forwarded packets
    int         ack    = -1;  // ACK field
    std::string data;
};

// ================== Helper Functions =====================

static bool extractStringField(const std::string& header,
                               const char* key,
                               std::string& value) {
    size_t idx = header.find(key);
    if (idx == std::string::npos) {
        return false;
    }

    idx += std::strlen(key);
    size_t end = header.find(';', idx);
    if (end == std::string::npos) {
        end = header.size();
    }

    value = header.substr(idx, end - idx);
    return true;
}

static bool extractIntField(const std::string& header,
                            const char* key,
                            int& value) {
    std::string tmp;
    if (!extractStringField(header, key, tmp)) {
        return false;
    }
    try {
        value = std::stoi(tmp);
    } catch (...) {
        return false;
    }
    return true;
}

static MeshPacket parsePacket(const String& rawStr) {
    MeshPacket packet;
    std::string raw(rawStr.c_str());

    // Split header and payload at ";DATA="
    size_t dataPos = raw.find(";DATA=");
    std::string headerPart;

    if (dataPos == std::string::npos) {
        headerPart = raw;
        packet.data.clear();
    } else {
        headerPart = raw.substr(0, dataPos);
        packet.data = raw.substr(dataPos + 6); // length of ";DATA="
    }

    // Extract all known fields
    if (!extractStringField(headerPart, "TYPE=", packet.type)) {
        packet.valid = false;
        return packet;
    }

    extractIntField(headerPart, "SRC=",  packet.src);
    extractIntField(headerPart, "DST=",  packet.dst);
    extractIntField(headerPart, "SEQ=",  packet.seq);
    extractIntField(headerPart, "TTL=",  packet.ttl);
    extractIntField(headerPart, "RANK=", packet.rank);
    extractIntField(headerPart, "ORIG=", packet.orig);
    extractIntField(headerPart, "ACK=",  packet.ack);

    packet.valid = true;
    return packet;
}

static std::string buildMessage(const char* type,
                                int src,
                                int dst,
                                int seq,
                                int rank = -1,
                                int ttl = -1,
                                int orig = -1,
                                int ack = -1,
                                const std::string& data = "") {
    std::string msg = "TYPE=";
    msg += type;
    msg += ";SRC=";
    msg += std::to_string(src);
    msg += ";DST=";
    msg += std::to_string(dst);
    msg += ";SEQ=";
    msg += std::to_string(seq);

    if (rank >= 0) {
        msg += ";RANK=";
        msg += std::to_string(rank);
    }
    if (ttl >= 0) {
        msg += ";TTL=";
        msg += std::to_string(ttl);
    }
    if (orig >= 0) {
        msg += ";ORIG=";
        msg += std::to_string(orig);
    }
    if (ack >= 0) {
        msg += ";ACK=";
        msg += std::to_string(ack);
    }
    if (!data.empty()) {
        msg += ";DATA=";
        msg += data;
    }

    return msg;
}

static void pruneSeenCache() {
    auto now = std::chrono::steady_clock::now();
    
    for (auto it = seenPackets.begin(); it != seenPackets.end(); ) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - it->second).count();
        
        if (elapsed > DEDUP_TIMEOUT_MS) {
            it = seenPackets.erase(it);
        } else {
            ++it;
        }
    }
}

static bool hasSeenPacket(int origId, int seqNum) {
    PacketKey key = {origId, seqNum};
    return seenPackets.find(key) != seenPackets.end();
}

static void addToSeenCache(int origId, int seqNum) {
    PacketKey key = {origId, seqNum};
    seenPackets[key] = std::chrono::steady_clock::now();
}

static std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::tm tm_now;
    localtime_r(&time_t_now, &tm_now);
    
    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    
    return oss.str();
}

static void initCSV() {
    // Open in append mode
    csvFile.open(CSV_FILENAME, std::ios::app);
    
    if (!csvFile.is_open()) {
        std::printf("[CSV] Failed to open %s\n", CSV_FILENAME);
        return;
    }
    
    // Check if file is empty (new file)
    csvFile.seekp(0, std::ios::end);
    if (csvFile.tellp() == 0) {
        // Write header
        csvFile << "Timestamp,NodeID,SoilTemp_C,AirTemp_C,Humidity_%,Light_Lux,Moisture_%,RSSI_dBm,SNR_dB,Sequence,Hops\n";
        csvFile.flush();
        csvHeaderWritten = true;
        std::printf("[CSV] Created new file with header: %s\n", CSV_FILENAME);
    } else {
        csvHeaderWritten = true;
        std::printf("[CSV] Appending to existing file: %s\n", CSV_FILENAME);
    }
}

static void parseAndLogSensorData(const std::string& data, int origId, int seq, 
                                   float rssi, float snr, int hops) {
    if (!csvFile.is_open()) {
        std::printf("[CSV] File not open, skipping log\n");
        return;
    }
    
    // Parse sensor data format: ID:5,ST:24.50,AT:22.30,H:45.60,L:1234.56,M:67
    int nodeId = -1;
    float soilTemp = 0.0f;
    float airTemp = 0.0f;
    float humidity = 0.0f;
    float light = 0.0f;
    int moisture = 0;
    
    // Simple parser
    std::istringstream iss(data);
    std::string token;
    
    while (std::getline(iss, token, ',')) {
        size_t colonPos = token.find(':');
        if (colonPos == std::string::npos) continue;
        
        std::string key = token.substr(0, colonPos);
        std::string value = token.substr(colonPos + 1);
        
        try {
            if (key == "ID") {
                nodeId = std::stoi(value);
            } else if (key == "ST") {
                soilTemp = std::stof(value);
            } else if (key == "AT") {
                airTemp = std::stof(value);
            } else if (key == "H") {
                humidity = std::stof(value);
            } else if (key == "L") {
                light = std::stof(value);
            } else if (key == "M") {
                moisture = std::stoi(value);
            }
        } catch (...) {
            // Skip malformed values
        }
    }
    
    // Use origId if nodeId wasn't parsed
    if (nodeId == -1) {
        nodeId = origId;
    }
    
    // Write to CSV
    std::string timestamp = getCurrentTimestamp();
    csvFile << timestamp << ","
            << nodeId << ","
            << std::fixed << std::setprecision(2)
            << soilTemp << ","
            << airTemp << ","
            << humidity << ","
            << light << ","
            << moisture << ","
            << rssi << ","
            << snr << ","
            << seq << ","
            << hops << "\n";
    
    csvFile.flush();  // Ensure data is written immediately
    
    std::printf("[CSV] Logged data from node %d\n", nodeId);
}

static bool transmitAck(int dataSeq, int senderId) {
    // ACK should be sent back to the immediate sender (repeater)
    const std::string ackPayload = buildMessage(
        "ACK",
        SINK_ID,          // src
        senderId,         // dst (the repeater who sent us the data)
        ++ackSeq,         // our ack sequence
        -1,               // rank (not needed in ACK)
        -1,               // ttl (not needed in ACK)
        -1,               // orig (not needed in ACK)
        dataSeq,          // ACK=dataSeq (acknowledging this sequence)
        ""                // no data payload
    );

    String ackStr(ackPayload.c_str());
    int state = radio.transmit(ackStr);

    if (state == RADIOLIB_ERR_NONE) {
        std::printf("[ACK] Sent to node %d (acking seq=%d)\n", senderId, dataSeq);
        return true;
    }

    std::printf("[ACK] Failed to send (code=%d)\n", state);
    return false;
}

static void transmitBeacon() {
    // Add small random jitter to avoid collision
    int jitter = rand() % BEACON_JITTER_MS;
    
    std::string payload = buildMessage(
        "BEACON",
        SINK_ID,          // src
        -1,               // dst (broadcast)
        ++beaconSeq,      // seq
        0,                // rank (sink is rank 0)
        -1,               // ttl
        -1,               // orig
        -1,               // ack
        ""                // no data
    );

    // Small delay for jitter
    hal->delay(jitter);

    String beaconStr(payload.c_str());
    int state = radio.transmit(beaconStr);

    if (state == RADIOLIB_ERR_NONE) {
        std::printf("[BEACON] Sent rank=0 seq=%d\n", beaconSeq);
    } else {
        std::printf("[BEACON] Failed (code=%d)\n", state);
    }
}

static void handlePacket(const String& raw, float rssi, float snr) {
    MeshPacket packet = parsePacket(raw);

    if (!packet.valid) {
        std::printf("[RX] Unparsed: %s\n", raw.c_str());
        return;
    }

    // Determine originator (for deduplication)
    int originatorId = (packet.orig >= 0) ? packet.orig : packet.src;

    std::printf("[RX] type=%s src=%d dst=%d seq=%d ttl=%d orig=%d\n",
                packet.type.c_str(),
                packet.src,
                packet.dst,
                packet.seq,
                packet.ttl,
                packet.orig);
    std::printf("     rssi=%.1f dBm snr=%.1f dB\n", rssi, snr);

    // Handle DATA or FLOOD packets
    if (packet.type == "DATA" || packet.type == "FLOOD") {
        // Check if this is a duplicate
        if (hasSeenPacket(originatorId, packet.seq)) {
            std::printf("[DEDUP] Already processed from orig=%d seq=%d\n",
                       originatorId, packet.seq);
            // Still send ACK in case previous ACK was lost
            if (packet.src >= 0 && packet.seq >= 0) {
                transmitAck(packet.seq, packet.src);
            }
            return;
        }

        // Add to seen cache
        addToSeenCache(originatorId, packet.seq);

        // Display the sensor data
        if (!packet.data.empty()) {
            std::printf("     DATA: %s\n", packet.data.c_str());
            
            // Calculate hops (TTL decreases as packet travels)
            int hops = 0;
            if (packet.type == "FLOOD" && packet.ttl >= 0) {
                // Assume MAX_TTL was 3 (from sensor code)
                hops = 3 - packet.ttl;
            } else if (packet.type == "DATA") {
                // Direct or one-hop transmission
                hops = (packet.orig >= 0 && packet.orig != packet.src) ? 2 : 1;
            }
            
            // Log to CSV
            parseAndLogSensorData(packet.data, originatorId, packet.seq, rssi, snr, hops);
        }

        // Send ACK back to immediate sender
        if (packet.src >= 0 && packet.seq >= 0) {
            transmitAck(packet.seq, packet.src);
        } else {
            std::printf("[ACK] Skipped: missing src or seq\n");
        }
    }
    // Handle BEACON packets (for monitoring network state)
    else if (packet.type == "BEACON") {
        std::printf("     BEACON from node %d rank=%d\n", packet.src, packet.rank);
    }

    std::printf("\n");
}

// ================== Main =====================

int main() {
    std::printf("=== SX1262 Mesh Sink Node (Rank 0) ===\n");
    std::printf("[INIT] Initializing receiver...\n");

    // Seed random for beacon jitter
    srand(time(NULL));

    // Initialize radio with matching parameters
    int state = radio.begin(
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

    radio.setCRC(true);
    
    std::printf("[INIT] Radio configured:\n");
    std::printf("       Freq: %.2f MHz\n", LORA_FREQ_MHZ);
    std::printf("       BW: %.1f kHz\n", LORA_BW_KHZ);
    std::printf("       SF: %d\n", LORA_SF);
    std::printf("       CR: 4/%d\n", LORA_CR);
    std::printf("       Power: %d dBm\n", LORA_PWR_DBM);
    std::printf("[INIT] Ready. Listening for packets...\n\n");

    auto lastBeacon = std::chrono::steady_clock::now();
    auto lastCachePrune = std::chrono::steady_clock::now();

    while (true) {
        auto now = std::chrono::steady_clock::now();
        
        // Periodic beacon transmission
        auto beaconElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - lastBeacon).count();
        
        if (beaconElapsed >= BEACON_INTERVAL_MS) {
            transmitBeacon();
            lastBeacon = now;
        }

        // Periodic cache pruning (every 10 seconds)
        auto pruneElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - lastCachePrune).count();
        
        if (pruneElapsed >= 10000) {
            pruneSeenCache();
            lastCachePrune = now;
        }

        // Receive packets
        String rx;
        state = radio.startReceive();
        
        if (state != RADIOLIB_ERR_NONE) {
            std::printf("[RX] Start receive failed (code=%d)\n", state);
            hal->delay(100);
            continue;
        }

        // Wait for packet with timeout
        hal->delay(RX_TIMEOUT_MS);
        
        state = radio.readData(rx);

        if (state == RADIOLIB_ERR_NONE) {
            float rssi = radio.getRSSI();
            float snr = radio.getSNR();
            handlePacket(rx, rssi, snr);
        }
        else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            // Timeout is expected, continue
        }
        else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            std::printf("[RX] CRC mismatch\n");
        }
        else {
            std::printf("[RX] Receive failed (code=%d)\n", state);
        }
    }

    return 0;
}
