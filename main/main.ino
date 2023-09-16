///////////////////
//SAROS_TestFlight_Main
//Version: 2.1
//Date: 09/15/2023
//Author: Tristan McGinnis & Sam Quartuccio
//Use: Main source code for SAROS test board
///////////////////

// Imports:
#include "SAROS_Util.h"


//Board Details
String ID = "STX";

//Packet Values
String packet;
int utc_hr, utc_min, utc_sec;
int mis_hr, mis_min;
double mis_time;
long int packetCt = 0;
int gp_sats;
double t_temp, humidity;
int16_t pd1, pd2, pd3, pd4;
long gp_lat, gp_lon, gp_alt;
static int32_t b_temp, b_humidity, b_press, b_gas;



//BNO Setup
//uint16_t BNO055_SAMPLERATE_DELAY_MS = 5;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1); // passed Wire into BNO055
double sea_level = 1013.25;
//

//BME680 Setup
//Adafruit_BME680 bme = Adafruit_BME680(&Wire1);
BME680_Class BME680;
//Bme68x bme;//bme object for BOSH Bme6 library



//Humidity Sensor
Adafruit_SHT4x sht4 = Adafruit_SHT4x(); //Humidity Sensor Setup


//GPS Setup
SFE_UBLOX_GNSS gps; 
//static const uint32_t GPSBaud = 38400;

//ADC to I2C ADS1015
ADS1015 ADS(0x49, &Wire1);
//Adafruit_ADS1015 ADS;

//


//General Variables
unsigned int lastBlink = 0; //Last time LED Blinked
int LEDS = 0; //LED status
unsigned int lastPoll = 0; //Last time polling major sensors
unsigned int lastShort = 0; //last time polling PDs only
unsigned int lastGPS = 0; //last GPS poll

//Used for SPI SD Logger
const int _MISO = 8;
const int _MOSI = 11;
const int _CS = 9;
const int _SCK = 10;
//File dataFile = SD.open("example.txt", FILE_WRITE);
int fileCt = 0;



void setup() {
  //dataFile.close();
  boolean setDynamicModel(dynModel newDynamicModel = DYN_MODEL_AIRBORNE4g, uint16_t maxWait = 1100);
  //uint8_t dynamicModelTest = getDynamicModel(uint16_t maxWait = 1100); // Get the dynamic model - returns 255 if the sendCommand fails
  
  Serial.begin(115200);//USB Interface
  pinMode(25, OUTPUT);//onboard pico LED
  
  setWire1(3,2);//(sda, scl)
  Wire1.begin();
  //Wire1.setClock(500000);
  
  //Setup for SPI SD Logger
  SPI1.setRX(_MISO);
  SPI1.setTX(_MOSI);
  SPI1.setSCK(_SCK);
  SPI1.setCS(_CS);

  digitalWrite(25, HIGH);
  delay(2000);
  digitalWrite(25, LOW);


  if (!SD.begin(_CS, SPI1)) {
    Serial.println("SD initialization failed!");
    //return;
  }else{
    Serial.println("SD initialization done.");
  }

  ID = "ST" + String(getBoardID()); //Get assigned board ID from id.txt on SD card

  //Increment file name id to generate different output files upon reset
  for(int i = 0; i < 20; i++)
  {
    String stringPrint = "";
    String fileName = "data_out_"+String(i)+".txt";
    if (SD.exists(fileName)) {
      stringPrint = fileName + " already exists";
      Serial.println(stringPrint);

    } else {
      stringPrint = fileName + " does not exist. Creating: data_out_" + String(i);
      Serial.println(stringPrint);
      fileCt = i;
      break;
    }
    if(i == 20)
    {
      Serial.println("File limit reached!");
      fileCt = 99;
    }
  }

  

  Serial2.setRX(5);
  Serial2.setTX(4);
  //set Pins for GPS, UART line begins during test
  //Serial2.begin(GPSBaud);//GPS interface

  pinMode(26, INPUT);//set INPUT pin mode for thermistor
  analogReadResolution(10);//up analog read resolution to 12 bit- TO BE REPLACED BY I2C ADC Converter
  
  // Start Up Sequence
  digitalWrite(25, HIGH);//gpio 2/3
  delay(2000);
  digitalWrite(25, LOW);
  delay(500);

  Serial.println("-----------------------"); 
  Serial.println("Start Up Sequence"); 
  Serial.println("-----------------------"); 
  


  //  BNO055 Check
  if (!bno.begin()){
    Serial.println("BNO055\t[ ]");
  }else
  {
    Serial.println("BNO055\t[X]");
  }
  


  //BME680 Check
  if(!BME680.begin(2000000, 0))
  {
    Serial.println("BME680\t[ ]");
  }else
  {
    Serial.println("BME680\t[X]");
    BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
    BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
    BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
    BME680.setIIRFilter(IIR4);
    BME680.setGas(0,0);
  }


  //  BME680 Check
  /*
  if (!bme.begin()){
    Serial.println("BME680\t[ ]");
  }else
  {
    Serial.println("BME680\t[X]");
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(0,0); // 320*C for 150 ms
    //bme.setSensorMode(BME680_PARALLEL_MODE);
    //bme.setODR(BME68X_ODR_NONE);
    //bme68x_set_op_mode(1);
  }
  */
  

  //  SHT4 Check 
  if (!sht4.begin(&Wire1)){
    Serial.println("SHT4X\t[ ]");
  }else
  {
    Serial.println("SHT4X\t[X]");
    sht4.setPrecision(SHT4X_HIGH_PRECISION);
  }

  //  ADS1015 Check
  if (!ADS.begin()){//for adafruit, set ADS.begin(0x49, &Wire1)
    Serial.println("ADS1015\t[ ]");
  }else
  {
    Serial.println("ADS1015\t[X]");
    ADS.setDataRate(7);//4000 for adafruit library, 7 for other
    //Serial.println("ADS_DR: "+String(ADS.getDataRate()));
    ADS.setMode(0);
    ADS.readADC(0);
  }

  


  //  GPS Check
  //Serial2.begin(9600);
  gps.begin(Wire1, 0x42);
  if (gps.getLatitude() != 0)
  {

    //Serial.println("GPS on 9600, switching to 38400");//38400 example suggestion
    //gps.setSerialRate(38400);
    Serial.println("NEO-M9N\t[X]");
    gps.setNavigationFrequency(30);
    gps.setI2COutput(COM_TYPE_UBX);
  }else
  {
    Serial.println("NEO-M9N\t[]");
  }

  /*delay(2000);
  Serial2.begin(38400);
  if (gps.begin(Serial2))
  {
    Serial.println("GPS on 38400");
    gps.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
    gps.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    gps.saveConfiguration(); //Save the current settings to flash and BBR
  }
  */


  //Cycle BME to remove initial garbage data
  /*
  for(int i = 0; i < 5; i++)
  {
    bme.performReading();
  }
  //Get Starting Pressure
  bme.performReading();
  sea_level = bme.pressure / 100.0;
  double temp_alt = bme.readAltitude(sea_level);
  Serial.println("Start Pressure/Alt: "+ String(sea_level) +","+String(bme.readAltitude(sea_level)));
  */
  //Cycle BME to remove initial garbage data
  
  for(int i = 0; i < 5; i++)
  {
    BME680.getSensorData(b_temp, b_humidity, b_press, b_gas);
    Serial.println("CLEARING...");
    delay(100);
  }
  
  /*
  while(1)
  {
    BME680.getSensorData(b_temp, b_humidity, b_press, b_gas);
    String testS = String(b_press/100.0);
    Serial.println(String(testS));
  }
*/


  Serial.println("-----------------------"); 
  Serial.println("Start Up Sequence Complete"); 
  Serial.println("-----------------------"); 
}

//henry.sun@yic.com

void loop() {
  String fName = "data_out_" + String(fileCt)+".txt"; //File name chosen based on last created file
  File dataFile = SD.open(fName, FILE_WRITE); //Open data output file
  
  while(mis_time <= 21600)//run for 6 hours
  {
    
    //LED Blinking
    /*
    if(millis()%500 == 0)
    {
      if(LEDS == 0)
      {
        digitalWrite(25, HIGH);
        LEDS = 1;
      }else
      {
        digitalWrite(25, LOW);
        LEDS = 0;
      }
    }*/

    mis_time = millis()/1000.0; //get mission time (system clock time)

    ADS.setGain(0);
    
    pd1 = ADS.readADC(0); //read photodiode 1
    pd2 = ADS.readADC(1); //read photodiode 2
    pd3 = ADS.readADC(2); //read photodiode 3
    pd4 = ADS.readADC(3); //read photodiode 4
    

    //pd1 = ADS.readADC_SingleEnded(0);
    //pd2 = ADS.readADC_SingleEnded(1);
    //pd2 = ADS.readADC_SingleEnded(2);
    //pd3 = ADS.readADC_SingleEnded(3);

    //t_temp = (1.0/(log((200000.0/((1024.0/analogRead(26)) - 1))/200000)/3892.0 + 1.0/(25 + 273.15))) - 273.15; //Calculation for Thermistor temperature value
    t_temp = analogRead(26);

    if(threadFunc(500, millis() , &lastPoll))//Run large-format packet every 500ms
    {
      digitalWrite(25, LOW);

      packetCt++;
      //bme.performReading();
      BME680.getSensorData(b_temp, b_humidity, b_press, b_gas);
      String bme_data = String(b_temp/100.0)+","+String(b_press/100.0) + "," +String(b_humidity);
      
      
      gp_lat = gps.getLatitude();
      gp_lon = gps.getLongitude();
      gp_sats = gps.getSIV();
      gp_alt = gps.getAltitude();
      utc_hr = gps.getHour();
      utc_min = gps.getMinute();
      utc_sec = gps.getSecond();
      

      sensors_event_t humidity, temp;
      sht4.getEvent(&humidity, &temp);


      //Large-Format 4hz packet
      packet = ID + "," + String(packetCt) + "," + String(mis_time) + "," + String(pd1)+","+String(pd2) + "," + String(pd3) +","+ String(pd4) +",";
      packet += String(t_temp) + "," + String(utc_hr) + ":" + String(utc_min) + ":" + String(utc_sec) + ",";
      packet += String(gp_lat)+","+String(gp_lon)+","+String(gp_sats)+","+String(gp_alt)+",";
      packet += String(bme_data)+","+String(humidity.relative_humidity)+",";
      packet += readBno(bno, 1) +","+readBno(bno, 2)+","+readBno(bno, 3)+","+readBno(bno, 4);
      Serial.println(packet);
      dataFile.println(packet);
      dataFile.flush();
    }else
    {
      //Small-Format no limit packet
      if(mis_time <= 14400.00)//Only run for 4 hours
      {
        digitalWrite(25, HIGH);
        packetCt++;
        packet = String(ID)+","+String(packetCt)+","+ String(mis_time) +","+String(pd1)+","+String(pd2) + "," + String(pd3)+ ","+String(pd4)+"," + String(t_temp) + ",,,,,,,,,,,,,,,,";
        //Serial.println(packet);
        dataFile.println(packet);
        //dataFile.flush();
      
      }
    }
  }



}
