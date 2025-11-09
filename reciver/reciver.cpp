#include <RadioLib.h>
#include "hal/RPi/PiHal.h"

PiHal* hal = new PiHal(0);                // SPI channel 0 (CE0)
SX1262 radio = new Module(hal, 8, 25, 23, 24);  
// NSS=8, DIO1=25, NRST=23, BUSY=24

int main() {
  printf("[SX1262] Initializing receiver...\n");

  int state = radio.begin(923.2, 500.0, 7, 5, 0x12, 22, 8);
  // freq MHz, BW kHz, SF, CR(4/5), syncWord, power, preamble len

  if (state != RADIOLIB_ERR_NONE) {
    printf("Init failed, code %d\n", state);
    return 1;
  }
  printf("Init success! Listening for packets...\n\n");

  while (true) {
    String str;
    state = radio.receive(str);

    if (state == RADIOLIB_ERR_NONE) {
      printf("[RX] %s\n", str.c_str());
      printf("     RSSI: %.1f dBm | SNR: %.1f dB\n\n", radio.getRSSI(), radio.getSNR());
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

