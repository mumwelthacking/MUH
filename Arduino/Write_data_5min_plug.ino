/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor
  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!
  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <Streaming.h>

//SD
#include <SD.h>
//BME680
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
//BH1750
#include <BH1750.h>
//SPS30
#include <sps30.h>


// LoRaWAN
#include <MKRWAN.h>
#include "arduino_secrets.h"
lora_band region = EU868;
LoRaModem modem(Serial1);

//SD
File myFile;

//BME680
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

//SEN0232
#define SoundSensorPin A1  //this pin read the analog voltage from the sound level meter
#define VREF  5.0  //voltage on AREF pin,default:operating voltage

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK);

//BH1750
BH1750 lightSensor = BH1750();

#pragma pack(push, 1)
struct Measurement {
  uint8_t v;

  float pm_mass_1p0;
  float pm_mass_2p5;
  float pm_mass_4p0;
  float pm_mass_10p0;
  float pm_count_0p5;
  float pm_count_1p0;
  float pm_count_2p5;
  float pm_count_4p0;
  float pm_count_10p0;

  float temperature_c;
  uint32_t pressure_pa;
  float humidity_relpct;
  uint32_t gas_resistance_ohm;

  uint16_t light_lux;

  float sound_dba;
};
#pragma pack(pop)

void setup() {
  //SD
  !SD.begin(4) ;

  // LoRaWAN
  if (!modem.begin(region)) {
    Serial.println("Failed to start module");
    while (1) {}
  };
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());
  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    Serial.println("Something went wrong; are you indoor? Move near a window and retry");
    while (1) {}
  }
  Serial.println("Successfully joined the network!");
  Serial.println("Enabling ADR and setting medium spreading factor");
  modem.setADR(true);
  modem.dataRate(3);

  //SPS30
  int16_t ret; //return values, genutzt um fehler abzufangen
  Serial.begin(9600);
  delay(2000);
  sensirion_i2c_init(); //initialisierung
  //BME680
  //BH1750
  lightSensor.begin();
  while (sps30_probe() != 0) {
    Serial.println("SPS sensor nicht gefunden..");
    delay(1000);
  }
  Serial.println("SPS sensor gefunden!");

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop() {
  Measurement current_measurement = {};

  //SPS30
  int16_t ret; //return values, genutzt um fehler abzufangen
  struct sps30_measurement sps_out; //entält unsere Messdaten
  ret = sps30_start_measurement();
  delay(5000);
  ret = sps30_read_measurement(&sps_out);

  current_measurement.pm_mass_1p0 = sps_out.mc_1p0;
  current_measurement.pm_mass_2p5 = sps_out.mc_2p5;
  current_measurement.pm_mass_4p0 = sps_out.mc_4p0;
  current_measurement.pm_mass_10p0 = sps_out.mc_10p0;
  current_measurement.pm_count_0p5 = sps_out.nc_0p5;
  current_measurement.pm_count_1p0 = sps_out.nc_1p0;
  current_measurement.pm_count_2p5 = sps_out.nc_2p5;
  current_measurement.pm_count_4p0 = sps_out.nc_4p0;
  current_measurement.pm_count_10p0 = sps_out.nc_10p0;

  sps30_stop_measurement();

  //BME680
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  delay(50); 
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  current_measurement.temperature_c = bme.temperature;
  current_measurement.pressure_pa = bme.pressure;
  current_measurement.humidity_relpct = bme.humidity;
  current_measurement.gas_resistance_ohm = bme.gas_resistance;

  //Light
  current_measurement.light_lux = lightSensor.getLightIntensity();

  // Sound
  float voltageValue, dbValue;
  voltageValue = analogRead(SoundSensorPin) / 1024.0 * VREF;
  current_measurement.sound_dba = voltageValue * 50.0;  //convert voltage to decibel value

  // SD
  myFile = SD.open("Messung.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    myFile << millis() << ","
      << current_measurement.pm_mass_1p0 << ","
      << current_measurement.pm_mass_2p5 << ","
      << current_measurement.pm_mass_4p0 << ","
      << current_measurement.pm_mass_10p0 << ","
      << current_measurement.pm_count_0p5 << ","
      << current_measurement.pm_count_1p0 << ","
      << current_measurement.pm_count_2p5 << ","
      << current_measurement.pm_count_4p0 << ","
      << current_measurement.pm_count_10p0 << ","
      << current_measurement.temperature_c << ","
      << current_measurement.pressure_pa << ","
      << current_measurement.humidity_relpct << ","
      << current_measurement.gas_resistance_ohm << ","
      << current_measurement.light_lux << ","
      << current_measurement.sound_dba << "\r\n";
    // close the file:
    myFile.close();
  } else {
    Serial.println("error opening Messung.txt");
  }

  // LoRaWAN
  modem.beginPacket();
  modem.write((char*) &current_measurement, sizeof(Measurement));
  int err = modem.endPacket(false);
  if (err > 0) {
    Serial.println("Big success!");
  } else {
    Serial.println("Error");
  }

  delay(900000);
}
