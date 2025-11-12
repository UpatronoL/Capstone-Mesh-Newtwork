#include <RadioLib.h>
#include "hal/RPi/PiHal.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

// ================== Radio / Mesh Config ==================

static PiHal* hal = new PiHal(0);                  // SPI channel 0 (CE0)
                                                   // NSS=8, DIO1=25, NRST=23, BUSY=24
static SX1262 radio = new Module(hal, 8, 25, 23, 24);

static const int SINK_ID     = 0;                  // This node (base)
static const int REPEATER_ID = 1;                  // Upstream repeater ID

// ================== Packet Structure =====================

struct MeshPacket {
    bool        valid  = false;
    std::string type;       // TYPE=
    int         src    = -1; // SRC=
    int         dst    = -1; // DST=
    int         ttl    = -1; // TTL=
    int         seq    = -1; // SEQ=
    int         rank   = -1; // RANK=
    int         orig   = -1; // ORIG=
    int         target = -1; // TARGET=
    std::string payload;     // DATA=
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
        value = header.substr(idx);
    } else {
        value = header.substr(idx, end - idx);
    }

    // Trim whitespace
    const char* ws = " \t\n\r";
    size_t first = value.find_first_not_of(ws);
    if (first == std::string::npos) {
        value.clear();
        return false;
    }
    size_t last = value.find_last_not_of(ws);
    value = value.substr(first, last - first + 1);

    return !value.empty();
}

static bool extractIntField(const std::string& header,
        const char* key,
        int& value) {
    std::string number;
    if (!extractStringField(header, key, number)) {
        return false;
    }
    value = static_cast<int>(std::strtol(number.c_str(), nullptr, 10));
    return true;
}

static MeshPacket parsePacket(const String& raw) {
    MeshPacket packet;

    std::string rawStd = raw.c_str();
    std::string header = rawStd;

    // Split header / payload on DATA=
    size_t dataIdx = rawStd.find("DATA=");
    if (dataIdx != std::string::npos) {
        header        = rawStd.substr(0, dataIdx);
        packet.payload = rawStd.substr(dataIdx + 5);   // after "DATA="
    }

    // TYPE is mandatory for a "valid" parsed packet
    if (!extractStringField(header, "TYPE=", packet.type)) {
        return packet;
    }

    extractIntField(header, "SRC=",    packet.src);
    extractIntField(header, "DST=",    packet.dst);
    extractIntField(header, "TTL=",    packet.ttl);
    extractIntField(header, "SEQ=",    packet.seq);
    extractIntField(header, "RANK=",   packet.rank);
    extractIntField(header, "ORIG=",   packet.orig);
    extractIntField(header, "TARGET=", packet.target);

    packet.valid = true;
    return packet;
}

static std::string buildMessage(const std::string& type,
        int src,
        int dst,
        int ttl,
        int seq,
        int rank,
        int orig,
        int target,
        const std::string& payload) {
    std::string msg;

    msg.reserve(128 + payload.size()); // small optimization

    msg += "TYPE=" + type;
    msg += ";SRC=" + std::to_string(src);
    msg += ";DST=" + std::to_string(dst);

    if (ttl    >= 0) msg += ";TTL="    + std::to_string(ttl);
    if (seq    >= 0) msg += ";SEQ="    + std::to_string(seq);
    if (rank   >= 0) msg += ";RANK="   + std::to_string(rank);
    if (orig   >= 0) msg += ";ORIG="   + std::to_string(orig);
    if (target >= 0) msg += ";TARGET=" + std::to_string(target);

    msg += ";DATA=" + payload;
    return msg;
}

static bool transmitAck(int seq, int sensorId) {
    const std::string ackPayload = buildMessage(
            "ACK",
            SINK_ID,          // src
            REPEATER_ID,      // dst (assumed repeater)
            -1,               // ttl (unused)
            seq,              // seq
            0,                // rank (0 at sink)
            SINK_ID,          // orig
            sensorId,         // target
            "ok"              // payload
            );

    String ackStr(ackPayload.c_str());
    int state = radio.transmit(ackStr);

    if (state == RADIOLIB_ERR_NONE) {
        std::printf("[ACK] Sent to repeater for sensor %d (seq=%d)\n", sensorId, seq);
        return true;
    }

    std::printf("[ACK] Failed to send (code=%d)\n", state);
    return false;
}

static void handlePacket(const String& raw) {
    MeshPacket packet = parsePacket(raw);

    if (!packet.valid) {
        std::printf("[RX] Unparsed: %s\n", raw.c_str());
        return;
    }

    std::printf("[RX] type=%s src=%d dst=%d seq=%d ttl=%d\n",
            packet.type.c_str(),
            packet.src,
            packet.dst,
            packet.seq,
            packet.ttl);
    std::printf("     data=%s\n", packet.payload.c_str());

    // For data-carrying frames, emit an ACK back into mesh
    if (packet.type == "DATA" || packet.type == "FLOOD") {
        const int sensorId =
            (packet.orig >= 0) ? packet.orig :
            (packet.src  >= 0) ? packet.src  :
            -1;

        if (sensorId >= 0 && packet.seq >= 0) {
            transmitAck(packet.seq, sensorId);
        } else {
            std::printf("[ACK] Skipped: missing sensorId or seq\n");
        }
    }
}

// ================== Main =====================

int main() {
    std::printf("[SX1262] Initializing receiver...\n");

    // freq MHz, BW kHz, SF, CR(4/5), syncWord, power dBm, preamble length
    int state = radio.begin(923.2, 500.0, 7, 5, 0x12, 22, 8);

    if (state != RADIOLIB_ERR_NONE) {
        std::printf("[SX1262] Init failed (code=%d)\n", state);
        return 1;
    }

    radio.setCRC(true);
    radio.setRxTimeout(2000);  // 2s timeout

    std::printf("[SX1262] Init OK. Listening for packets...\n\n");

    while (true) {
        String rx;
        state = radio.receive(rx);

        if (state == RADIOLIB_ERR_NONE) {
            handlePacket(rx);
            std::printf("     RSSI=%.1f dBm | SNR=%.1f dB\n\n",
                    radio.getRSSI(),
                    radio.getSNR());
        }
        else if (state == RADIOLIB_ERR_RX_TIMEOUT ||
                state == RADIOLIB_ERR_CRC_MISMATCH) {
            continue;
        }
        else {
            std::printf("[RX] Receive failed (code=%d)\n", state);
        }
    }

    return 0;
}
