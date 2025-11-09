#include <RadioLib.h>

// ===== Wio-E5 internal radio (STM32WL) =====
STM32WLx_Module wl;
STM32WLx radio(&wl);

// ===== Wio-E5 RF switch: PA4/PA5 (REQUIRED) =====
// RX:  PA4=1, PA5=0
// TX:  PA4=0, PA5=1  (LP/HP same route on this module)
static const uint32_t RFSW_PINS[] = { PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC };
static const Module::RfSwitchMode_t RFSW_TABLE[] = {
  { STM32WLx::MODE_IDLE,  { LOW,  LOW  } },
  { STM32WLx::MODE_RX,    { HIGH, LOW  } },
  { STM32WLx::MODE_TX_LP, { LOW,  HIGH } },
  { STM32WLx::MODE_TX_HP, { LOW,  HIGH } },
  END_OF_MODE_TABLE,
};

// ===== LoRa params — must match your transmitter =====
static const float   FREQ_MHZ  = 923.2;   // JP example: set to your legal channel
static const float   BW_KHZ    = 500.0;   // FAST
static const uint8_t SF        = 5;       // FAST
static const uint8_t CR        = 5;       // 4/5
static const uint8_t SYNCWORD  = 0x12;    // public LoRa
static const int8_t  PWR_DBM   = 14;      // not important for RX, but set anyway
static const uint16_t PREAMBLE = 8;
static const float    TCXO_V   = 1.6;
static const bool     USE_LDO  = false;

const char* errStr(int16_t c) {
  switch (c) {
    case RADIOLIB_ERR_NONE: return "OK";
    case RADIOLIB_ERR_RX_TIMEOUT: return "RX_TIMEOUT";
    case RADIOLIB_ERR_CRC_MISMATCH: return "CRC_MISMATCH";
    case RADIOLIB_ERR_INVALID_FREQUENCY: return "INVALID_FREQ";
    case RADIOLIB_ERR_INVALID_BANDWIDTH: return "INVALID_BW";
    case RADIOLIB_ERR_INVALID_SPREADING_FACTOR: return "INVALID_SF";
    case RADIOLIB_ERR_INVALID_CODING_RATE: return "INVALID_CR";
    case RADIOLIB_ERR_CHIP_NOT_FOUND: return "CHIP_NOT_FOUND";
    default: return "ERR_OTHER";
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serail) {}
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
  Serial.println("Applying RF switch table");
  int16_t state = radio.begin(FREQ_MHZ, BW_KHZ, SF, CR, SYNCWORD, PWR_DBM, PREAMBLE, TCXO_V, USE_LDO);
  if(state != RADIOLIB_ERR_NONE) {
    Serial.println(F("LoRa init failed! Check wiring/core/frequency."));
    while (true) { delay(1000); }
  }

  radio.setCRC(true);
  Serial.println("Setup Complete");
}

void loop() {
  // put your main code here, to run repeatedly:

}
