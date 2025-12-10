# 🌐 LoRa Mesh Network for Environmental Sensing  
*A lightweight, self-healing, multi-hop LoRa mesh designed for agricultural sensor networks.*

---

## 📘 Overview

This repository implements a **custom LoRa-based mesh communication protocol** designed for large greenhouse environments with heavy metal structures and intermittent line-of-sight.

The network consists of:

- **Sensor Nodes** → Gather environmental measurements and push data toward the sink  
- **Repeaters** → Form a multi-hop routing tree, forward packets, assist joins  
- **Sink (Receiver)** → Collects data at the gateway (STM32WLE5 or Raspberry Pi)

The system provides **automatic discovery**, **rank-based routing**, **active probing**, **collision-resistant timing**, and **robust CRC-protected binary packets**.

---

## 🧩 Features

### ✔ Mesh Architecture  
- Multi-hop routing using **ranks** (hop distance from sink)  
- Automatic parent selection using RSSI + rank scoring  
- Hysteresis-based parent switching  
- Proactive **join assist** on repeaters for unjoined neighbors  

### ✔ Reliable Data Delivery  
- Binary LoRa frames with CRC-8  
- Hop-by-hop ACKs  
- Retries + failover parent switching  
- Optional flood-based recovery  

### ✔ Discovery & Join Process  
- Sensors use **active PROBE_REQ / PROBE_RESP** to find repeaters  
- Repeaters reply with rank and RSSI  
- Sensors choose the best parent and complete JOIN_REQ ↔ JOIN_ACK handshake  

### ✔ Optimized Timing  
- SF5/BW500 → extremely short preambles  
- Eliminates expensive Serial prints during tight RX windows  
- Carefully inserted delays prevent RX/TX overlap issues

### ✔ Data Handling  
- Sensors send compact **binary payloads** (14 bytes)  
- Repeaters decode/forward efficiently  
- Sink logs to **CSV** (Pi) or USB Serial CSV (Wio-E5)  

---

## 📁 Repository Structure

```
mesh-network/
├── docs
│   └── UMLdiagram.png
├── firmware
│   ├── repeater
│   │   └── repeater.ino
│   ├── sensor
│   │   └── sensor.ino
│   └── sink_wio
│       └── receiver_wio.ino
├── host
│   └── receiver_pi
│       ├── CMakeLists.txt
│       └── receiver.cpp
└── README.md
```

---

## 🔌 Communication Protocol

### 📨 Packet Types

| Type | Direction | Purpose |
|------|-----------|---------|
| `PKT_BEACON` | Sink/Repeater → Sensor | Advertise rank and presence |
| `PKT_PROBE_REQ` | Sensor → Repeater | Active discovery request |
| `PKT_PROBE_RESP` | Repeater → Sensor | Discovery reply with rank/RSSI |
| `PKT_JOIN_REQ` | Sensor → Parent | Join tree request |
| `PKT_JOIN_ACK` | Parent → Sensor | Parent confirmation |
| `PKT_DATA` | Sensor/Repeater → Upstream | Binary payload forwarding |
| `PKT_ACK` | Parent → Child | Hop acknowledgment |

All packets include **CRC-8 (poly 0x07)** for validation.

---

## 📡 Mesh Operation Flow

### 1. **Sensor Boot → Discovery**
1. Broadcast `PROBE_REQ` every 3 seconds  
2. Collect `PROBE_RESP` packets  
3. Compute neighbor score = `RSSI – rank × weight`  
4. Pick best parent candidate  

### 2. **Join Handshake**
1. Sensor sends `JOIN_REQ(parentId)`  
2. Parent sends several `JOIN_ACK` retries  
3. Sensor returns a **final ACK**  
4. Parent is confirmed → sensor enters JOINED mode  

### 3. **Data Uplink**
1. Prepare binary 14-byte sensor payload  
2. Send `PKT_DATA` → parent  
3. Parent returns `PKT_ACK`  
4. Repeaters decrement TTL and forward  
5. Sink logs packet to CSV  

### 4. **Failover**
If ACK not received:
- Retry until `MAX_DATA_RETRIES`  
- Invalidate parent  
- Re-discover and re-join  

---

## 🔧 Firmware Components

---

### **Sensor Node (`sensor.ino`)**
- Active probing for neighbor discovery  
- Rank-scored parent selection  
- Complete JOIN handshake  
- Binary payload generation (fixed 14 bytes)  
- Hop-based TTL forwarding logic  
- Failover if ACK missing  

Key behaviors:
- Avoids long Serial prints during join  
- Caches uplink state for retries  
- Automatically re-enters discovery if parent dies  

---

### **Repeater Node (`repeater.ino`)**
- Maintains neighbor table (id, rank, RSSI, age)  
- Computes own rank based on best upstream repeater  
- Implements parent-hysteresis to avoid oscillations  
- Handles JOIN_REQ / JOIN_ACK handshake  
- Forwards DATA up the tree with TTL decrement  
- Dedup cache to prevent loops  
- Periodic BEACONs advertise rank  

**JOIN Assist Mode:**  
If repeater sees an unjoined node (`rank=255`), it temporarily enters JOIN_HANDLING to help sensors complete handshake.

**Forward Queue:**  
Decouples DATA forwarding from beacon timing to avoid collision.

---

### **Sink Node (Wio-E5 or Raspberry Pi)**

#### **Wio-E5 Sink (`receiver_wio.ino`)**
- Uses RadioLib STM32WLx driver  
- Sends rank-0 BEACONs  
- Receives and decodes binary DATA packets  
- CSV logging via USB Serial  
- Dedup cache based on (origSrc, origSeq)

#### **Raspberry Pi Sink (`receiver.cpp`)**
- SPI-connected SX1262 using PiHAL  
- Parses binary or ASCII messages (depending on version)  
- Logs measurements to CSV:  
  ```
  Timestamp, NodeID, SoilTemp, AirTemp, Humidity, Lux, Moisture, RSSI, SNR, Seq, Hops
  ```  
- Returns ACK back into mesh  

---

## 🖼 UML Communication Diagram

![Mesh Network Communication Sequence](docs/UMLdiagram.png)

---

## 📊 Example Logs

### Repeater
```
[NB] 2 rank=1 rssi=-103
[PARENT] Switch 1 -> 2
[TX] DATA -> parent ttl=9
[ACK] Local ACK sent to child 5
```

### Sensor
```
[PROBE] RESP from 2 rank=1 rssi=-98
[DISCOVERY] Selected repeater 2
[JOIN] JOIN_ACK received from 2
[UPLINK] Sent DATA seq=34
```

### Sink
```
[RX] BINARY DATA os=5 oq=34 hops=2
[CSV] Logged binary data from node 5
```

---

## 📌 Future Work
- Downlink configuration frames  
- Energy-aware sleep scheduling  
- Congestion-based parent switching  
- Multi-parent redundancy  

---
