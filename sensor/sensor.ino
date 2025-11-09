#include <RadioLib.h>
#include <Wire.h>
#include <AHT20.h>
#include <DallasTemperature.h>
#include <BH1750.h>
#include <OneWire.h>

// ------------------- User Pin Configuration -------------------
#define I2C_SDA_PIN PB7
#define I2C_SCL_PIN PB6
#define ONE_WIRE_BUS PB0
#define SOIL_MOISTURE_PIN PA1

// ------------------- LoRa Parameters (FASTEST SETUP) -------------------
static const float   FREQ_MHZ  = 923.2;  // Japan band example
static const float   BW_KHZ    = 500.0;  // Maximum bandwidth
static const uint8_t SF        = 5;      // Lower SF = faster
static const uint8_t CR        = 5;      // 4/5
static const uint8_t SYNCWORD  = 0x12;   // Public LoRa
static const int8_t  PWR_DBM   = 22;     // Max TX power
static const uint16_t PREAMBLE = 8;
static const float    TCXO_V   = 1.6;
static const bool     USE_LDO  = false;

const int DEVICE_ID = 5;

STM32WLx_Module wl;
STM32WLx radio(&wl);

// RF switch table (PA4 / PA5) - mandatory for Wio-E5 Mini
static const uint32_t RFSW_PINS[] = { PA4, PA5, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC };
static const Module::RfSwitchMode_t RFSW_TABLE[] = {
  { STM32WLx::MODE_IDLE,  { LOW,  LOW  } },
  { STM32WLx::MODE_RX,    { HIGH, LOW  } },
  { STM32WLx::MODE_TX_LP, { LOW,  HIGH } },
  { STM32WLx::MODE_TX_HP, { LOW,  HIGH } },
  END_OF_MODE_TABLE,
};

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
AHT20 aht20;
BH1750 lightMeter;

String getSensorReadings() {
  sensors.requestTemperatures();
  float soilTemp = sensors.getTempCByIndex(0);
  int i = 0;
  while (soilTemp == 85.0f && i < 100) {
    delay(1);
    soilTemp = sensors.getTempCByIndex(0);
    i++;
  }

  float airTemp = aht20.getTemperature();
  float humidity = aht20.getHumidity();
  float lux = lightMeter.readLightLevel();
  int rawMoisture = analogRead(SOIL_MOISTURE_PIN);
  int moisturePercent = map(rawMoisture, 0, 4095, 0, 100);

  char msg[128];
  snprintf(msg, sizeof(msg),
           "ID:%d,ST:%.2f,AT:%.2f,H:%.2f,L:%.2f,M:%d",
           DEVICE_ID, soilTemp, airTemp, humidity, lux, moisturePercent);
  return String(msg);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println();
  Serial.println(F("=== Wio-E5 Mini Fast LoRa Transmitter Test ==="));

  // I2C setup
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  // Sensors
  Serial.println(F("Initializing sensors..."));
  if (!aht20.begin()) Serial.println(F("AHT20 init failed."));
  sensors.begin();
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
    Serial.println(F("BH1750 init failed."));

  // LoRa radio
  radio.setRfSwitchTable(RFSW_PINS, RFSW_TABLE);
  Serial.println(F("Applying RF switch table..."));

  int16_t state = radio.begin(FREQ_MHZ, BW_KHZ, SF, CR, SYNCWORD, PWR_DBM, PREAMBLE, TCXO_V, USE_LDO);
  Serial.print(F("radio.begin -> ")); Serial.println(state);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.println(F("LoRa init failed! Check wiring/core/frequency."));
    while (true) { delay(1000); }
  }

  radio.setCRC(true);
  Serial.println(F("Setup complete"));
}

void loop() {
  // put your main code here, to run repeatedly:

}
