//PINS
//Screen: 4 (SDA), 15 (SCL), 16 (RST)
//General I2C (BME280): 21 (SDA), 22 (SCL)
//Volume: 36, 37, 38
//Relays: 17, 2, 23
//CO2: 13 (RX TTGO), 25 (TX TTGO)

//BEFORE LOADING THE CODE
//Remember to change NWKSKEY, APPSKEY and DEVADDR to whichever values are set on your TheThingsNetwork application and device
//If you are in range of a multiple channel gateway, uncomment the definitions for all of the other channels

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <BluetoothSerial.h>

static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const PROGMEM u1_t APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const u4_t DEVADDR = 0x00000000 ;

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

#define SOUND_MEASUREMENT_LENGTH 0.05
#define SOUND_INTERVAL 1
#define TX_INTERVAL 60

unsigned long lastSound = 0;
unsigned long lastTX = 0;

const unsigned int microphonePins[3] = {36, 37, 38};
const double mCoeff[3] = {15.377934, 9.484695, 9.117650}; //Coefficients m and n come from linear regression so that dB=m*ln(V)+n
const double nCoeff[3] = {118.312656, 86.271022, 59.830157};
const double maximumDecibels[3] {250, 80, 60};  //Less than the theoretical maximum (using linear regression) each can measure, except for the first one

const unsigned int relayPins[3] = {17, 2, 23};
const float triggeringVolume[3] = {0, 65, 85};

const byte disableABC[] = {0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6};
const byte readCO2[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

double volume;
double cumulativeVolume;
int volumeReadings;

float temperature;
float pressure;
float humidity;

int CO2;
byte bufferCO2[9];

String text;
static uint8_t textBytes[256];

const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 14,
  .dio = {26, 33, 32},
};

Adafruit_SSD1306 display(128, 64, &Wire, 16);
Adafruit_BME280 bme280;
BluetoothSerial bluetooth;

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }

  Serial.println();
}

float analogReadVolts(unsigned int pin, unsigned int readings = 100) {
  float values[] = {0,    1,   77,  197,  318,  438,  564,  694,  811,  931, 1069, 1180, 1306, 1435, 1562, 1680, 1806, 1924, 2053, 2174, 2294, 2418, 2535, 2674, 2801, 2927, 3071, 3221, 3389, 3587, 3798, 4031, 4095, 4095};
  float volts[] =  {0,  0.1,  0.2,  0.3,  0.4,  0.5,  0.6,  0.7,  0.8,  0.9,  1.0,  1.1,  1.2,  1.3,  1.4,  1.5,  1.6,  1.7,  1.8,  1.9,  2.0,  2.1,  2.2,  2.3,  2.4,  2.5,  2.6,  2.7,  2.8,  2.9,  3.0,  3.1,  3.2,  3.3};

  long x = 0;

  for (int i = 0; i < readings; i++) {
    x += analogRead(pin);
    delayMicroseconds(500);
  }

  x /= readings;

  for (int i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
    if (x <= values[i]) {

      if (i == 0) {
        return volts[0];
      }

      if (values[i - 1] == values[i]) {
        return (volts[i - 1] + volts[i]) / 2;
      }

      return (x - values[i - 1]) / (values[i] - values[i - 1]) * (volts[i] - volts[i - 1]) + volts[i - 1];
    }
  }
  return volts[sizeof(volts) / sizeof(volts[0]) - 1];
}

void setup() {
  Serial.begin(115200);

  bluetooth.begin("Classroom quality sensor");
  bluetooth.setPin("2468");

  Serial1.begin(9600, SERIAL_8N1, 13, 25);

  Wire.begin(4, 15);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Starting...");
  display.display();

  delay(500);

  Wire.begin(21, 22);
  bme280.begin();
  Serial1.write(disableABC, 9);

#ifdef VCC_ENABLE
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  os_init();
  LMIC_reset();

#ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  /*
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);
  */
#elif defined(CFG_us915)
  LMIC_selectSubBand(1);
#endif

  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7, 14);

  double readings[sizeof(microphonePins) / sizeof(microphonePins[0])];
  double peakToPeak[sizeof(microphonePins) / sizeof(microphonePins[0])];
  double peakToPeakDecibels[sizeof(microphonePins) / sizeof(microphonePins[0])];
  double signalMin[sizeof(microphonePins) / sizeof(microphonePins[0])] = {1e10, 1e10, 1e10};
  double signalMax[sizeof(microphonePins) / sizeof(microphonePins[0])] = {0, 0, 0};

  Wire.begin(21, 22);

  temperature = bme280.readTemperature();
  pressure = bme280.readPressure() / 100;
  humidity = bme280.readHumidity();

  for (int i = 0; i < sizeof(microphonePins) / sizeof(microphonePins[0]); i++) {
    signalMin[i] = 1e10;
    signalMax[i] = 0;
  }

  lastSound = millis();

  while (!((millis() > lastSound + 1000 * SOUND_MEASUREMENT_LENGTH) && ((lastSound + 1000 * SOUND_MEASUREMENT_LENGTH > lastSound) || (millis() + 1000 * SOUND_MEASUREMENT_LENGTH > millis())))) {
    for (int i = 0; i < sizeof(microphonePins) / sizeof(microphonePins[0]); i++) {
      readings[i] = analogReadVolts(microphonePins[i], 1);

      if (readings[i] > signalMax[i]) {
        signalMax[i] = readings[i];
      }
      if (readings[i] < signalMin[i]) {
        signalMin[i] = readings[i];
      }
    }
  }

  for (int i = 0; i < sizeof(microphonePins) / sizeof(microphonePins[0]); i++) {
    peakToPeak[i] = signalMax[i] - signalMin[i];
    peakToPeakDecibels[i] = mCoeff[i] * log(peakToPeak[i]) + nCoeff[i];
  }

  for (int i = sizeof(microphonePins) / sizeof(microphonePins[0]) - 1; i >= 0; i--) {
    if (peakToPeakDecibels[i] <= maximumDecibels[i]) {
      volume = peakToPeakDecibels[i];
      break;
    }
  }

  byte bufferCO2[9];
  while (Serial1.available() > 0) {
    Serial1.read();
  }
  Serial1.write(readCO2, 9);
  Serial1.readBytes(bufferCO2, 9);
  CO2 = (unsigned int)(bufferCO2[2] & B00011111) * 256 + (unsigned int)bufferCO2[3];

  Serial.print("Time: ");
  Serial.print(millis());
  Serial.println(" ms");

  Serial.print("Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");

  Serial.print("Atmospheric pressure: ");
  Serial.print(pressure, 2);
  Serial.println(" hPa");

  Serial.print("Humidity: ");
  Serial.print(humidity, 2);
  Serial.println(" %");

  Serial.print("Volume: ");
  Serial.print(volume, 2);
  Serial.println(" dB");

  Serial.print("CO2: ");
  Serial.print(CO2);
  Serial.println(" ppm");

  Serial.println();



  bluetooth.print("Time: ");
  bluetooth.print(millis());
  bluetooth.println(" ms");

  bluetooth.print("Temperature: ");
  bluetooth.print(temperature, 2);
  bluetooth.println(" *C");

  bluetooth.print("Atmospheric pressure: ");
  bluetooth.print(pressure, 2);
  bluetooth.println(" hPa");

  bluetooth.print("Humidity: ");
  bluetooth.print(humidity, 2);
  bluetooth.println(" %");

  bluetooth.print("Volume: ");
  bluetooth.print(volume, 2);
  bluetooth.println(" dB");

  bluetooth.print("CO2: ");
  bluetooth.print(CO2);
  bluetooth.println(" ppm");

  bluetooth.println();

  Wire.begin(4, 15);

  unsigned long tempTX = lastTX;
  int dayTX = 0;
  int hourTX = 0;
  int minuteTX = 0;

  while (tempTX >= 86400E3) {
    tempTX -= 86400E3;
    dayTX++;
  }
  while (tempTX >= 3600E3) {
    tempTX -= 3600E3;
    hourTX++;
  }
  while (tempTX >= 60E3) {
    tempTX -= 60E3;
    minuteTX++;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Day " + String(dayTX) + " " + ((hourTX < 10) ? "0" : "") + String(hourTX) + ":" + ((minuteTX < 10) ? "0" : "") + String(minuteTX) + " " + String(volume, 2) + "dB");
  for (int i = 0; i < sizeof(microphonePins) / sizeof(microphonePins[0]); i++) {
    display.println("Pin " + String(microphonePins[i]) + ": " + String(peakToPeak[i] * 1000, 0) + "mV (" + String(peakToPeakDecibels[i], 0) + "dB)");
  }
  display.println(String(temperature, 2) + "*C " + String(humidity, 2) + "%");
  display.println(String(pressure, 2) + "hPa " + String(CO2) + "ppmCO2");
  display.display();
}

void loop() {
  if ((millis() > lastSound + 1000 * SOUND_INTERVAL) && ((lastSound + 1000 * SOUND_INTERVAL > lastSound) || (millis() + 1000 * SOUND_INTERVAL > millis()))) {
    lastSound = millis();

    double readings[sizeof(microphonePins) / sizeof(microphonePins[0])];
    double peakToPeak[sizeof(microphonePins) / sizeof(microphonePins[0])];
    double peakToPeakDecibels[sizeof(microphonePins) / sizeof(microphonePins[0])];
    double signalMin[sizeof(microphonePins) / sizeof(microphonePins[0])] = {1e10, 1e10, 1e10};
    double signalMax[sizeof(microphonePins) / sizeof(microphonePins[0])] = {0, 0, 0};

    for (int i = 0; i < sizeof(microphonePins) / sizeof(microphonePins[0]); i++) {
      signalMin[i] = 1e10;
      signalMax[i] = 0;
    }

    while (!((millis() > lastSound + 1000 * SOUND_MEASUREMENT_LENGTH) && ((lastSound + 1000 * SOUND_MEASUREMENT_LENGTH > lastSound) || (millis() + 1000 * SOUND_MEASUREMENT_LENGTH > millis())))) {
      for (int i = 0; i < sizeof(microphonePins) / sizeof(microphonePins[0]); i++) {
        readings[i] = analogReadVolts(microphonePins[i], 1);

        if (readings[i] > signalMax[i]) {
          signalMax[i] = readings[i];
        }
        if (readings[i] < signalMin[i]) {
          signalMin[i] = readings[i];
        }
      }
    }

    for (int i = 0; i < sizeof(microphonePins) / sizeof(microphonePins[0]); i++) {
      peakToPeak[i] = signalMax[i] - signalMin[i];
      peakToPeakDecibels[i] = mCoeff[i] * log(peakToPeak[i]) + nCoeff[i];
    }

    for (int i = sizeof(microphonePins) / sizeof(microphonePins[0]) - 1; i >= 0; i--) {
      if (peakToPeakDecibels[i] <= maximumDecibels[i]) {
        volume = peakToPeakDecibels[i];
        break;
      }
    }

    cumulativeVolume += volume;
    volumeReadings ++;

    for (int i = sizeof(microphonePins) / sizeof(microphonePins[0]) - 1; i >= 0; i--) {
      if (volume > triggeringVolume[i]) {
        for (int j = 0; j < sizeof(relayPins) / sizeof(relayPins[0]); j++) {
          pinMode(relayPins[j], OUTPUT);
          digitalWrite(relayPins[j], i == j);
        }
        break;
      }
    }
    display.fillRect(0, 8, 128, 24, 0);
    display.setCursor(0, 8);
    for (int i = 0; i < sizeof(microphonePins) / sizeof(microphonePins[0]); i++) {
      display.println("Pin " + String(microphonePins[i]) + ": " + String(peakToPeak[i] * 1000, 0) + "mV (" + String(peakToPeakDecibels[i], 0) + "dB)");
    }
    display.display();
  }
  if ((millis() > lastTX + 1000 * TX_INTERVAL) && ((lastTX + 1000 * TX_INTERVAL > lastTX) || (millis() + 1000 * TX_INTERVAL > millis()))) {
    lastTX = millis();

    Wire.begin(21, 22);

    temperature = bme280.readTemperature();
    pressure = bme280.readPressure() / 100;
    humidity = bme280.readHumidity();

    volume = cumulativeVolume / volumeReadings;
    cumulativeVolume = 0;
    volumeReadings = 0;

    byte bufferCO2[9];
    while (Serial1.available() > 0) {
      Serial1.read();
    }
    Serial1.write(readCO2, 9);
    Serial1.readBytes(bufferCO2, 9);
    CO2 = (unsigned int)(bufferCO2[2] & B00011111) * 256 + (unsigned int)bufferCO2[3];

    Serial.print("Time: ");
    Serial.print(millis());
    Serial.println(" ms");

    Serial.print("Temperature: ");
    Serial.print(temperature, 2);
    Serial.println(" °C");

    Serial.print("Atmospheric pressure: ");
    Serial.print(pressure, 2);
    Serial.println(" hPa");

    Serial.print("Humidity: ");
    Serial.print(humidity, 2);
    Serial.println(" %");

    Serial.print("Volume: ");
    Serial.print(volume, 2);
    Serial.println(" dB");

    Serial.print("CO2: ");
    Serial.print(CO2);
    Serial.println(" ppm"); bluetooth.print("Time: ");

    bluetooth.print(millis());
    bluetooth.println(" ms");

    bluetooth.print("Temperature: ");
    bluetooth.print(temperature, 2);
    bluetooth.println(" *C");

    bluetooth.print("Atmospheric pressure: ");
    bluetooth.print(pressure, 2);
    bluetooth.println(" hPa");

    bluetooth.print("Humidity: ");
    bluetooth.print(humidity, 2);
    bluetooth.println(" %");

    bluetooth.print("Volume: ");
    bluetooth.print(volume, 2);
    bluetooth.println(" dB");

    bluetooth.print("CO2: ");
    bluetooth.print(CO2);
    bluetooth.println(" ppm");

    bluetooth.println();

    text = "";
    text += (-80 <= temperature && temperature <= 80) ? String(temperature, 2) : "";
    text += "/";
    text += (0 <= pressure && pressure <= 2000) ? String(pressure, 2) : "";
    text += "/";
    text += (0 <= humidity && humidity <= 100) ? String(humidity, 2) : "";
    text += "/";
    text += (0 <= volume && volume <= 200) ? String(volume, 2) : "";
    text += "/";
    text += (0 < CO2 && CO2 <= 5000 && CO2 != 256) ? String(CO2) : "";

    Serial.print("Sending: ");
    Serial.println(text);

    for (unsigned int i = 0; i < min(text.length(), sizeof(textBytes) / sizeof(textBytes[0])); i++) {
      textBytes[i] = text.charAt(i);
    }

    if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      LMIC_setTxData2(1, textBytes, text.length(), 0);
      Serial.println(F("Packet queued"));
    }

    Serial.println();

    Wire.begin(4, 15);

    unsigned long tempTX = lastTX;
    int dayTX = 0;
    int hourTX = 0;
    int minuteTX = 0;

    while (tempTX >= 86400E3) {
      tempTX -= 86400E3;
      dayTX++;
    }
    while (tempTX >= 3600E3) {
      tempTX -= 3600E3;
      hourTX++;
    }
    while (tempTX >= 60E3) {
      tempTX -= 60E3;
      minuteTX++;
    }

    display.fillRect(0, 0, 128, 8, 0);
    display.setCursor(0, 0);
    display.println("Day " + String(dayTX) + " " + ((hourTX < 10) ? "0" : "") + String(hourTX) + ":" + ((minuteTX < 10) ? "0" : "") + String(minuteTX) + " " + String(volume, 2) + "dB");
    display.fillRect(0, 32, 128, 32, 0);
    display.setCursor(0, 32);
    display.println(String(temperature, 2) + "*C " + String(humidity, 2) + "%");
    display.println(String(pressure, 2) + "hPa " + String(CO2) + "ppmCO2");
    display.display();
  }

  os_runloop_once();
}
