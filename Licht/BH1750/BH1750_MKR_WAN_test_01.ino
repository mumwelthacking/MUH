#include <Wire.h>
#include <MKRWAN.h>
#include "arduino_secrets.h" 

#define BH1750_ADDR 0x23
static const int POWER_DOWN = 0x00; // No active state (TODO)
static const int POWER_ON   = 0x01; // Waiting for measurement command
static const int RESET      = 0x07; // Reset data register value (Only in power on mode)

// Start measurement at 1 lx resolution. Measurement Time is typically 120ms
static const int CONTINUOUS_H_RES_MODE_1 = 0x10;
// Start measurement at 0.5 lx resolution. Measurement Time is typically 120ms
static const int CONTINUOUS_H_RES_MODE_2 = 0x11;
// Start measurement at 4 lx resolution. Measurement Time is typically 16ms.
static const int CONTINUOUS_L_RES_MODE = 0x13;
// Start measurement at 1 lx resolution. Measurement Time is typically 120ms
// Device is automatically set to Power Down after measurement.
static const int ONE_TIME_H_RES_MODE_1 = 0x20;
// Start measurement at 0.5 lx resolution. Measurement Time is typically 120ms
// Automatically set to Power Down after measurement. --> Save Power
static const int ONE_TIME_H_RES_MODE_2 = 0x21;
// Start measurement at 1 lx resolution. Measurement Time is typically 120ms
// Automatically set to Power Down after measurement. --> Save Power
static const int ONE_TIME_L_RES_MODE = 0x23;
//Typical minimum wait times
static const float CONTINUOUS_MIN_WAIT_H_RES_1 = 0.12;
static const float CONTINUOUS_MIN_WAIT_H_RES_2 = 0.15;
static const float CONTINUOUS_MIN_WAIT_L_RES   = 0.02;
static const float ONE_TIME_MIN_WAIT_H_RES_1   = 0.2;
static const float ONE_TIME_MIN_WAIT_H_RES_2   = 0.2;
static const float ONE_TIME_MIN_WAIT_L_RES     = 0.03;

int buffer[2];
static const int LUX_BUF_LEN = 16;
uint16_t lux_buffer[LUX_BUF_LEN];
uint8_t lux_buffer_8[2*LUX_BUF_LEN];
unsigned int counter = 0;

// Select your region (AS923, AU915, EU868, KR920, IN865, US915, US915_HYBRID)
_lora_band region = EU868;

LoRaModem modem(Serial1);

int readBH1750(int address)
{
   int i=0;
   Wire.beginTransmission(address);
   Wire.requestFrom(address, 2);
   while(Wire.available() && i < 2)
   {
      buffer[i] = Wire.read();
      i++;
   }
   Wire.endTransmission(); 
   return i;
}
void initBH1750(int address)
{
  Wire.beginTransmission(address);
  Wire.write(0x10);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while (!Serial);
  if (!modem.begin(region)) {
    Serial.println("Failed to start module");
    while (1) {}
  };
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());

  Serial.print("Connecting...");
  Serial.flush();
  int connected = modem.joinOTAA(appEui, appKey);
  Serial.println(" Connecting try finished");
  Serial.flush();
  if (!connected) {
    Serial.println("Something went wrong; are you indoor? Move near a window and retry");
    Serial.flush();
    while (1) {}
  }
  Serial.println("Successfully joined the network!");

  Serial.println("Enabling ADR and setting low spreading factor");
  modem.setADR(true);
  modem.dataRate(5);
}

void loop() {
   uint8_t lux=0;
   initBH1750( BH1750_ADDR );
   delay(200);
   if( readBH1750( BH1750_ADDR ) == 2 )
   {
      lux = ((buffer[0]<<8)|buffer[1])/1.2;
      counter++;
      lux_buffer[LUX_BUF_LEN - counter]=lux;
   }
   if( counter == LUX_BUF_LEN )
   {
      for( byte j = 0; j < LUX_BUF_LEN; j++)
      {
         lux_buffer_8[2*j] = lux_buffer[j] & 0xFF;
         lux_buffer_8[2*j + 1] = lux_buffer[j] >> 8;
      }
      counter=0;
      modem.beginPacket();
      modem.write(lux_buffer_8, 2*LUX_BUF_LEN);
      int err = modem.endPacket(false);
      if (err > 0) {
         Serial.println("Big success!");
      } else {
         Serial.println("Error");
      }
   }

   Serial.println(lux);

   delay(1000); //* 60);
}
