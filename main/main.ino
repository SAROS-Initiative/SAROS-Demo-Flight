///////////////////
//SAROS_TestFlight_Main
//Version: 1.0
//Author: Tristan McGinnis & Sam Quartuccio
//Use: Main source code for SAROS test board
///////////////////

// Imports:
#include <Wire.h> 
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_BNO055.h> 
#include <utility/imumaths.h> 
#include "Adafruit_BMP3XX.h" 
#include "Adafruit_SHT4x.h" 
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>


// Set up for BNO, BMP, SHT, GPS
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // passed Wire into BNO055
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
TinyGPSPlus gps;
static const uint32_t GPSBaud = 4800;
SoftwareSerial ss(4, 5);


void setup() {
  // Set Serial
  Serial.begin(115200);
  
  // Set I2C0 for Wire (using GPIO pin #s)
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();
  // Set I2C1 for Wire1
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();
  // Set UART for Serial2
  Serial2.setRX(5);
  Serial2.setTX(4);
  Serial2.begin(GPSBaud);

  // Serial2.begin(115200);
  // Set UART for Serial2

  // Set SPI for Reader
  // pinMode(ss, OUTPUT);
  
  // Start Up Sequence
  delay(5000);
  Serial.println("Start Up Sequence Check"); 
  Serial.println("-----------------------"); 

  /* BNO055 Check */
  if (!bno.begin()){
    //while(1)
    {
    Serial.print("BNO055 not detected");
  }}
  Serial.println("Found BNO055 sensor");

  /* BMP388 Check */
  if (!bmp.begin_I2C(0x77, &Wire1)){
    while(1){
    Serial.print("BMP388 not detected");
  }}
  Serial.println("Found BMP388 sensor");

  /* SHT4 Check */
  if (!sht4.begin(&Wire1)){
    while(1){
    Serial.print("SHT4 not detected");
  }}
  Serial.println("Found SHT4x sensor");

  /* GPS Check */
  unsigned long YICTimeTest;
  if (Serial2.available() > 0)
  {
    Serial.println("Serial2 Available, encoding GPS");
    YICTimeTest = millis();
    do
    {
      gps.encode(Serial2.read());
      //Serial.print(".");
    }while (millis() - YICTimeTest < 5000);
    Serial.println("Encode Test Complete");
  }else
  {
    Serial.println("Serial2 NOT Available");
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected: check wiring.");
    while(true);
  }
  Serial.println("Found YIC sensor [Final Check Completed]");
}

void loop() {
  //displayInfo();
  //printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  
  int testSats = gps.satellites.value();
  float testLong = gps.location.lng();
  float testLat = gps.location.lat();
  gps.encode(Serial2.read());
  Serial.print(testSats);
  Serial.print("\t");
  Serial.print(testLat);
  Serial.print("\t");
  Serial.println(testLong);
  Serial.println(Serial2.read());
  delay(5000);
  
  // put your main code here, to run repeatedly:
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial2.available())
      gps.encode(Serial2.read());
  } while (millis() - start < ms);
}
