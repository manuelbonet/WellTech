//VERSION
//Date: 2020-05-31

//PINS
//Screen: 4 (SDA), 15 (SCL), 16 (RST)
//General I2C (BME280): 21 (SDA), 22 (SCL)
//OneWire: 17
//Wind speed: 36
//Wind direction: 37
//Correction angle potentiometer: 39
//Serial (CO2): 13 (RX), 25 (TX)

//BEFORE LOADING THE CODE
//Remember to change NWKSKEY, APPSKEY and DEVADDR to whichever values are set on your TheThingsNetwork application and device
//If you are in range of a multiple channel gateway, change MULTICHANNEL to 1
//Before using the values of decibels, remember to calibrate the microphones

#include <math.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <DallasTemperature.h>
#include <BluetoothSerial.h>

static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const PROGMEM u1_t APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

static const u4_t DEVADDR = 0x00000000 ;

#define MULTICHANNEL 0

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

#define WIND_INTERVAL 1
#define TX_INTERVAL 60

unsigned long lastWind = - WIND_INTERVAL * 1000;
unsigned long lastTX = - TX_INTERVAL * 1000;
unsigned long millisOverflows = -1;
bool sendValues = false;

const byte disableABC[] = {0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6};
const byte readCO2[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

float insideTemperature;
float temperature;
float apparentTemperature;
float dewPoint;
float pressure;
float insideHumidity;
float humidity;
float windSpeed;
float maxWindSpeed;
float windDirection;
float adjustedWindDirection;
float cumulativeWindX;
float cumulativeWindY;
float averageWindDirection;
float correctionAngle;
float CO2;

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
OneWire oneWire(17);
DallasTemperature ds18b20(&oneWire);
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
  Wire.begin(4, 15);
  Serial.begin(115200);
  Serial.println("Starting...");

  bluetooth.begin("Weather station");
  bluetooth.setPin("2468");

  Serial1.begin(9600, SERIAL_8N1, 13, 25);
  Serial1.write(disableABC, 9);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Starting...");
  display.display();

  Wire.begin(21, 22);
  bme280.begin();
  ds18b20.begin();

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
#if (MULTICHANNEL)
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);
#else
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(1, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(2, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(3, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(4, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(5, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(6, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(7, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  LMIC_setupChannel(8, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
#endif
#elif defined(CFG_us915)
  LMIC_selectSubBand(1);
#endif

  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7, 14);
}

void loop() {
  unsigned long millisNow = millis();

  if ((millisNow >= (unsigned long)(lastWind + 1000 * WIND_INTERVAL)) && (((unsigned long)(lastWind + 1000 * WIND_INTERVAL) > lastWind) || ((unsigned long)(millisNow + 1000 * WIND_INTERVAL) > millisNow))) {
    lastWind = millisNow;

    windSpeed = 43.2 * analogReadVolts(36);
    if (windSpeed > maxWindSpeed) {
      maxWindSpeed = windSpeed;
    }
    windDirection = round(6.95652173913 * analogReadVolts(37)) * 22.5;
    correctionAngle = floor(analogReadVolts(39) * 5) * 22.5; if (correctionAngle == 360) correctionAngle = 337.5;
    adjustedWindDirection = windDirection - correctionAngle; while (adjustedWindDirection >= 360) adjustedWindDirection -= 360;  while (adjustedWindDirection < 0) adjustedWindDirection += 360;
    cumulativeWindX += cos(adjustedWindDirection * PI / 180.0);
    cumulativeWindY += sin(adjustedWindDirection * PI / 180.0);

    Wire.begin(4, 15);

    display.fillRect(0, 32, 128, 16, 0);
    display.setCursor(0, 32);
    display.println(String(windSpeed, 2) + "km/h " + String(adjustedWindDirection) + "*");
    display.println("WCA: " + String(correctionAngle, 1) + "*");
    display.display();
  }

  if ((millisNow >= (unsigned long)(lastTX + 1000 * TX_INTERVAL)) && (((unsigned long)(lastTX + 1000 * TX_INTERVAL) > lastTX) || ((unsigned long)(millisNow + 1000 * TX_INTERVAL) > millisNow))) {
    if (millisNow < lastTX){
      millisOverflows++;
    }

    lastTX = millis();

    Wire.begin(21, 22);

    ds18b20.requestTemperatures();
    insideTemperature = bme280.readTemperature();
    temperature = ds18b20.getTempCByIndex(0);
    delay(50);
    pressure = bme280.readPressure() / 100;
    delay(50);
    insideHumidity = bme280.readHumidity();
    humidity = constrain(insideHumidity * exp((18.768 - insideTemperature / 234.5) * (insideTemperature / (257.1 + insideTemperature)) - (18.768 - temperature / 234.5) * (temperature / (257.1 + temperature))), 0, 100);
    dewPoint = 1.0 / 2.0 * (4379.991 - 234.5 * (log(humidity / 100) + (18.678 - temperature / 234.5) * (temperature / (257.14 + temperature))) - sqrt(pow(234.5 * (log(humidity / 100) + (18.678 - temperature / 234.5) * (temperature / (257.14 + temperature))) - 4379.991, 2) - 241197.32 * (log(humidity / 100) + (18.678 - temperature / 234.5) * (temperature / (257.14 + temperature)))));
    averageWindDirection = round(atan2(cumulativeWindY, cumulativeWindX) / PI * 180.0 / 22.5) * 22.5; while (averageWindDirection < 0) averageWindDirection += 360;
    if (temperature >= 26 && humidity >= 40) {
      apparentTemperature = -8.78469476 + 1.61139411 * temperature + 2.338548839 * humidity - 0.14611605 * temperature * humidity - 0.012308094 * pow(temperature, 2) - 0.016424828 * pow(humidity, 2) + 0.002211732 * pow(temperature, 2) * humidity + 0.00072546 * temperature * pow(humidity, 2) - 0.000003582 * pow(temperature, 2) * pow(humidity, 2);
    } else if (temperature <= 10 && maxWindSpeed >= 3.2) {
      apparentTemperature =  13.1267 + 0.6215 * temperature - 11.37 * pow(maxWindSpeed * 1.5, 0.16) + 0.3965 * temperature * pow(maxWindSpeed * 1.5, 0.16);
    } else {
      apparentTemperature = temperature;
    }

    byte bufferCO2[9];
    while (Serial1.available()) {
      Serial1.read();
    }
    Serial1.write(readCO2, 9);
    Serial1.readBytes(bufferCO2, 9);
    CO2 = (unsigned int)(bufferCO2[2] && B00011111) * 256 + (unsigned int)bufferCO2[3];

    uint64_t fullMillis = lastTX + 4294967296 * millisOverflows;
    unsigned long fullMillisA = fullMillis / 1E16;
    unsigned long fullMillisB = (fullMillis - fullMillisA * 1E16) / 1E8;
    unsigned long fullMillisC = fullMillis - fullMillisA - fullMillisB;

    uint64_t tempTX = fullMillis;
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

    Serial.print("Time: day ");
    Serial.print(dayTX);
    Serial.print(" ");
    if (hourTX < 10) {
      Serial.print("0");
    }
    Serial.print(hourTX);
    Serial.print(":");
    if (minuteTX < 10) {
      Serial.print("0");
    }
    Serial.print(minuteTX);
    Serial.print(" (");
    if (fullMillisA!=0){
      Serial.print(fullMillisA);
    }
    if (fullMillisB!=0){
      Serial.print(fullMillisB);
    } else if (fullMillisA!=0){
      Serial.print("00000000");
    }
    Serial.print(fullMillisC);
    Serial.println(" ms)");

    Serial.print("Inside temperature: ");
    Serial.print(insideTemperature, 2);
    Serial.println(" °C");

    Serial.print("Outside temperature: ");
    Serial.print(temperature, 2);
    Serial.println(" °C");

    Serial.print("Dew point: ");
    Serial.print(dewPoint, 2);
    Serial.println(" °C");

    Serial.print("Atmospheric pressure: ");
    Serial.print(pressure, 2);
    Serial.println(" hPa");

    Serial.print("Inside humidity: ");
    Serial.print(insideHumidity, 2);
    Serial.println(" %");

    Serial.print("Outside humidity: ");
    Serial.print(humidity, 2);
    Serial.println(" %");

    Serial.print("Maximum wind speed: ");
    Serial.print(maxWindSpeed, 2);
    Serial.println(" km/h");

    Serial.print("Average wind direction: ");
    Serial.print(averageWindDirection, 1);
    Serial.println("°");

    Serial.print("Apparent temperature: ");
    Serial.print(apparentTemperature, 2);
    Serial.println(" °C");

    Serial.print("CO2: ");
    Serial.print(CO2, 0);
    Serial.println(" ppm");

    bluetooth.print("Time: day ");
    bluetooth.print(dayTX);
    bluetooth.print(" ");
    if (hourTX < 10) {
      bluetooth.print("0");
    }
    bluetooth.print(hourTX);
    bluetooth.print(":");
    if (minuteTX < 10) {
      bluetooth.print("0");
    }
    bluetooth.print(minuteTX);
    bluetooth.print(" (");
    if (fullMillisA!=0){
      bluetooth.print(fullMillisA);
    }
    if (fullMillisB!=0){
      bluetooth.print(fullMillisB);
    } else if (fullMillisA!=0){
      bluetooth.print("00000000");
    }
    bluetooth.print(fullMillisC);
    bluetooth.println(" ms)");

    bluetooth.print("Inside temperature: ");
    bluetooth.print(insideTemperature, 2);
    bluetooth.println(" *C");

    bluetooth.print("Outside temperature: ");
    bluetooth.print(temperature, 2);
    bluetooth.println(" *C");

    bluetooth.print("Dew point: ");
    bluetooth.print(dewPoint, 2);
    bluetooth.println(" *C");

    bluetooth.print("Atmospheric pressure: ");
    bluetooth.print(pressure, 2);
    bluetooth.println(" hPa");

    bluetooth.print("Inside humidity: ");
    bluetooth.print(insideHumidity, 2);
    bluetooth.println(" %");

    bluetooth.print("Outside humidity: ");
    bluetooth.print(humidity, 2);
    bluetooth.println(" %");

    bluetooth.print("Maximum wind speed: ");
    bluetooth.print(maxWindSpeed, 2);
    bluetooth.println(" km/h");

    bluetooth.print("Average wind direction: ");
    bluetooth.print(averageWindDirection, 1);
    bluetooth.println("*");

    bluetooth.print("Apparent temperature: ");
    bluetooth.print(apparentTemperature, 2);
    bluetooth.println(" *C");

    bluetooth.print("CO2: ");
    bluetooth.print(CO2, 0);
    bluetooth.println(" ppm");
    bluetooth.println();

    text = "";
    text += (-80 <= temperature && temperature <= 80) ? String(temperature, 2) : "";
    text += "/";
    text += (-80 <= apparentTemperature && apparentTemperature <= 80) ? String(apparentTemperature, 2) : "";
    text += "/";
    text += (0 <= pressure && pressure <= 2000) ? String(pressure, 2) : "";
    text += "/";
    text += (0 <= humidity && humidity <= 100) ? String(humidity, 2) : "";
    text += "/";
    text += (-80 <= dewPoint && dewPoint <= 80) ? String(dewPoint, 2) : "";
    text += "/";
    text += String(maxWindSpeed, 2);
    text += "/";
    text += String(averageWindDirection, 1);
    text += "/";
    text += (0 < CO2 && CO2 <= 5000 && CO2 != 256) ? String(CO2, 0) : "";

    if (sendValues){
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
    } else {
      Serial.print("Not sending: ");
      Serial.println(text);
    }

    Serial.println();

    Wire.begin(4, 15);

    display.fillRect(0, 0, 128, 32, 0);
    display.setCursor(0, 0);
    display.println("Day " + String(dayTX) + " " + ((hourTX < 10) ? "0" : "") + String(hourTX) + ":" + ((minuteTX < 10) ? "0" : "") + String(minuteTX));
    display.println(String(temperature, 2) + "*C (" + String(apparentTemperature) + "*Cap)");
    display.println(String(humidity, 2) + "% " + String(dewPoint, 2) + "*Cdew");
    display.println(String(pressure, 2) + "hPa " + String(CO2, 0) + "ppm");
    display.display();

    Wire.begin(21, 22);

    maxWindSpeed = 0;
    cumulativeWindX = 0;
    cumulativeWindY = 0;

    sendValues = true;
  }

  os_runloop_once();
}
