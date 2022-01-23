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

void setup() {
  //SD
  !SD.begin(4) ;

  
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
    float values[16];

    //SPS30
    int16_t ret; //return values, genutzt um fehler abzufangen
    struct sps30_measurement sps_out; //entält unsere Messdaten
    ret = sps30_start_measurement();
    delay(5000);
    ret = sps30_read_measurement(&sps_out);

    values[0] = sps_out.mc_1p0;
    values[1] = sps_out.mc_2p5;
    values[2] = sps_out.mc_4p0;
    values[3] = sps_out.mc_10p0;
    values[4] = sps_out.nc_0p5;
    values[5] = sps_out.nc_1p0;
    values[6] = sps_out.nc_2p5;
    values[7] = sps_out.nc_4p0;
    values[8] = sps_out.nc_10p0;
    values[9] = sps_out.typical_particle_size;
    //for (byte i = 0; i < (sizeof(measurement_values) / sizeof(measurement_values[0])); i++) {
    //  measurement_values[i] = 

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
    values[10] = bme.temperature;
    values[11] = bme.pressure;
    values[12] = bme.humidity;
    values[13] = bme.gas_resistance;

    //Light
    values[14] = lightSensor.getLightIntensity();

    // Sound
    float voltageValue, dbValue;
    voltageValue = analogRead(SoundSensorPin) / 1024.0 * VREF;
    dbValue = voltageValue * 50.0;  //convert voltage to decibel value
    value[15] = dbValue;

    //SD
    myFile = SD.open("Messung.txt", FILE_WRITE);

    // if the file opened okay, write to it:
    if (myFile) {
      myFile.println(millis());
      for (byte i = 0; i < (sizeof(values) / sizeof(values[0])); i++) {
     
    
    
     
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

      sps30_stop_measurement();
    
  delay(350000);
}
