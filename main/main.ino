///////////////////
//SAROS_TestFlight_Main
//Version: 1.1
//Date: 7/7/2023
//Author: Tristan McGinnis & Sam Quartuccio
//Use: Main source code for SAROS test board
///////////////////

// Imports:
#include "SAROS_Util.h"

//Board Details
String ID = "ST1";

//Packet Values
String packet;
int utc_hr, utc_min, utc_sec;
int mis_hr, mis_min;
double mis_time;
long int packetCt = 0;
int gp_sats;
double rel_altitude, t_temp, b_pres, b_temp, humidity;
double pd1, pd2;
long gp_lat, gp_lon, gp_alt;


//BNO Setup
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire); // passed Wire into BNO055
double sea_level = 1013.25;
//


Adafruit_BMP3XX bmp; //BMP Setup
Adafruit_SHT4x sht4 = Adafruit_SHT4x(); //Humidity Sensor Setup


//GPS Setup
SFE_UBLOX_GNSS gps; 
//static const uint32_t GPSBaud = 38400;

boolean setDynamicModel(dynModel newDynamicModel = DYN_MODEL_AIRBORNE4g, uint16_t maxWait = 1100);
uint8_t getDynamicModel(uint16_t maxWait = 1100); // Get the dynamic model - returns 255 if the sendCommand fails
//

//General Variables
unsigned int lastBlink = 0; //Last time LED Blinked
unsigned int lastPoll = 0; //Last time polling major sensors
unsigned int lastShort = 0; //last time polling PDs only


void setup() {
  Serial.begin(115200);//USB Interface
  pinMode(25, OUTPUT);//onboard pico LED
  
  setWire0(1, 0); //set I2C0 for Wire -- SCL 1 SDA 0
  Wire.begin();
  setWire1(3, 2);//set I2C1 for Wire1 -- SCL 3 SDA 2
  Wire1.begin();
  
  Serial1.setRX(13);
  Serial1.setTX(12);
  Serial1.begin(250000);//Adafruit OpenLog Interface (TEMP)

  
  Serial2.setRX(5);
  Serial2.setTX(4);
  //set Pins for YIC, UART line begins during test
  //Serial2.begin(GPSBaud);//YIC interface

  pinMode(28, INPUT);//set INPUT pin mode for thermistor
  analogReadResolution(12);//up analog read resolution to 12 bit

  
  // Start Up Sequence
  digitalWrite(25, HIGH);
  delay(2000);
  digitalWrite(25, LOW);
  delay(500);

  Serial.println("-----------------------"); 
  Serial.println("Start Up Sequence"); 
  Serial.println("-----------------------"); 
  Serial1.println("-----------------------"); 
  Serial1.println("Start Up Sequence"); 
  Serial1.println("-----------------------"); 


  //  BNO055 Check
  if (!bno.begin()){
    Serial.println("BNO055\t[ ]");
  }else
    Serial.println("BNO055\t[X]");

  //  BMP388 Check
  if (!bmp.begin_I2C(0x77, &Wire1)){
    Serial.println("BMP388\t[ ]");
  }else
  {
    Serial.println("BMP388\t[X]");
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }
  

  //  SHT4 Check 
  if (!sht4.begin(&Wire1)){
    Serial.println("SHT4X\t[ ]");
  }else
    Serial.println("SHT4X\t[X]");
    sht4.setPrecision(SHT4X_HIGH_PRECISION);

  //  YIC Check
  Serial2.begin(9600);
  if (gps.begin(Serial2))
  {
    Serial.println("YIC on 9600, switching to 38400");//38400 example suggestion
    gps.setSerialRate(38400);
  }
  delay(2000);
  Serial2.begin(38400);
  if (gps.begin(Serial2))
  {
    Serial.println("YIC on 38400");
    gps.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
    gps.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    gps.saveConfiguration(); //Save the current settings to flash and BBR
  }

  //Cycle BMP
  for(int i = 0; i < 10; i++)
  {
    bmp.performReading();
  }

  //Get Starting Pressure
  bmp.performReading();
  sea_level = bmp.readPressure() / 100.0;
  double temp_alt = bmp.readAltitude(sea_level);
  Serial.println("Start Pressure/Alt: "+ String(sea_level) +","+String(bmp.readAltitude(sea_level)));
  Serial1.println("Start Pressure/Alt: "+ String(sea_level)+","+String(bmp.readAltitude(sea_level)));
  
  Serial.println("-----------------------"); 
  Serial.println("Start Up Sequence Complete"); 
  Serial.println("-----------------------"); 
  Serial1.println("-----------------------"); 
  Serial1.println("Start Up Sequence Complete"); 
  Serial1.println("-----------------------"); 
}

void loop() {
  
  if(threadFunc(250, millis() , &lastPoll))
  {
    packetCt++;
    bmp.performReading();

    sensors_event_t humidity, temp;
    sht4.getEvent(&humidity, &temp);
    if(Serial2.available())
    {
      gp_lat = gps.getLatitude();
      gp_lon = gps.getLongitude();
      gp_sats = gps.getSIV();
      utc_hr = gps.getHour();
      utc_min = gps.getMinute();
      utc_sec = gps.getSecond();
    }
    
    t_temp = (1.0/(log((200000.0/((4095.0/analogRead(28)) - 1))/200000)/3892.0 + 1.0/(25 + 273.15))) - 273.15;

    pd1 = analogRead(26);
    pd2 = analogRead(27);
    
    //mis_time = millis()/1000.00;
    mis_time = millis()/1000.0;
    
    packet = ID + "," + String(mis_time) +","+ String(utc_hr) + ":" + String(utc_min) + ":" + String(utc_sec) + ",";
    packet += String(packetCt)+","+String(bmp.readAltitude(sea_level))+","+String(t_temp)+",";
    packet += String(gp_lat)+","+String(gp_lon)+","+String(gp_sats)+","+String(gp_alt)+","+String(bmp.pressure/100.0)+",";
    packet += String(bmp.temperature)+","+String(humidity.relative_humidity)+","+String(pd1)+","+String(pd2)+",";
    packet += readBno(bno, 1) +","+readBno(bno, 2)+","+readBno(bno, 3)+","+readBno(bno, 4);
    Serial.println(packet);
    Serial1.println(packet);
  }else
  {
    delay(50);
    packetCt++;
    packet = String(ID)+","+String(mis_time)+","+String(packetCt)+","+String(pd1)+","+String(pd2);
    Serial.println(packet);
    Serial1.println(packet);
  }

  




}
