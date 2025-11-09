#include <RadioLib.h>
#include "hal/RPi/PiHal.h"

#include <cstring>
#include <cstdlib>
#include <string>

PiHal* hal = new PiHal(0);
SX1262 radio = new Module(hal, 8, 25, 23, 24);

static const int SINK_ID = 0;
static const int REPEATER_ID = 1;

uint16_t sinkSequence = 0;

struct MeshPacket {
  bool valid = false;
  std::string type;
  int src = -1;
  bool hasSrc = false;
  int dst = -1;
  bool hasDst = false;
  int seq = 0;
  bool hasSeq = false;
  int ttl = 0;
  bool hasTtl = false;
  int ack = 0;
  bool hasAck = false;
  int orig = -1;
  bool hasOrig = false;
  std::string data;
  bool hasData = false;
};

uint16_t nextSequence() {
  return ++sinkSequence;
}

MeshPacket parsePacket(const String& raw) {
  MeshPacket packet;
  std::string rawStd = raw.c_str();
  std::string header = rawStd;
  std::size_t dataIdx = rawStd.find(";DATA=");
  if (dataIdx != std::string::npos) {
    header = rawStd.substr(0, dataIdx);
    packet.data = rawStd.substr(dataIdx + 6);
    packet.hasData = true;
  } else if (rawStd.rfind("DATA=", 0) == 0) {
    packet.data = rawStd.substr(5);
    packet.hasData = true;
    header.clear();
  }

  std::size_t start = 0;
  while (start < header.size()) {
    std::size_t end = header.find(';', start);
    if (end == std::string::npos) {
      end = header.size();
    }
    std::string token = header.substr(start, end - start);
    if (!token.empty()) {
      std::size_t eq = token.find('=');
      if (eq != std::string::npos) {
        std::string key = token.substr(0, eq);
        std::string value = token.substr(eq + 1);
        if (key == "TYPE") {
          packet.type = value;
        } else if (key == "SRC") {
          packet.src = std::strtol(value.c_str(), nullptr, 10);
          packet.hasSrc = true;
        } else if (key == "DST") {
          packet.dst = std::strtol(value.c_str(), nullptr, 10);
          packet.hasDst = true;
        } else if (key == "SEQ") {
          packet.seq = std::strtol(value.c_str(), nullptr, 10);
          packet.hasSeq = true;
        } else if (key == "TTL") {
          packet.ttl = std::strtol(value.c_str(), nullptr, 10);
          packet.hasTtl = true;
        } else if (key == "ACK") {
          packet.ack = std::strtol(value.c_str(), nullptr, 10);
          packet.hasAck = true;
        } else if (key == "ORIG") {
          packet.orig = std::strtol(value.c_str(), nullptr, 10);
          packet.hasOrig = true;
        }
      }
    }
    start = end + 1;
  }

  if (!packet.type.empty() && packet.hasSrc && packet.hasDst) {
    packet.valid = true;
  }
  return packet;
}

String buildAckMessage(int seq) {
  String msg = "TYPE=ACK;SRC=";
  msg += SINK_ID;
  msg += ";DST=";
  msg += REPEATER_ID;
  msg += ";SEQ=";
  msg += nextSequence();
  msg += ";ACK=";
  msg += seq;
  return msg;
}

void sendAckFor(int seq) {
  String ack = buildAckMessage(seq);
  int state = radio.transmit(ack);
  if (state == RADIOLIB_ERR_NONE) {
    printf("[ACK] Sent ack for seq %d\n", seq);
  } else {
    printf("[ACK] Failed to send ack (%d)\n", state);
  }
}

int main() {
  printf("[SX1262] Initializing receiver...\n");

  int state = radio.begin(923.2, 500.0, 7, 5, 0x12, 22, 8);
  if (state != RADIOLIB_ERR_NONE) {
    printf("Init failed, code %d\n", state);
    return 1;
  }
  radio.setCRC(true);
  printf("Init success! Listening for packets...\n\n");

  while (true) {
    String str;
    state = radio.receive(str);

    if (state == RADIOLIB_ERR_NONE) {
      MeshPacket packet = parsePacket(str);
      if (packet.valid) {
        printf("[RX] type=%s src=%d dst=%d seq=%d\n",
               packet.type.c_str(), packet.src, packet.dst, packet.seq);
        if (packet.hasOrig) {
          printf("     origin sensor: %d\n", packet.orig);
        }
        if (packet.hasData) {
          printf("     data: %s\n", packet.data.c_str());
        }
        printf("     RSSI: %.1f dBm | SNR: %.1f dB\n", radio.getRSSI(), radio.getSNR());

        if (packet.type == "DATA" && packet.dst == SINK_ID && packet.hasSeq) {
          sendAckFor(packet.seq);
        }
      } else {
        printf("[RX] Raw payload: %s\n", str.c_str());
      }
    }
    else if (state == RADIOLIB_ERR_RX_TIMEOUT || state == RADIOLIB_ERR_CRC_MISMATCH) {
      continue;
    }
    else {
      printf("Receive failed, code %d\n", state);
    }
  }

  return 0;
}
