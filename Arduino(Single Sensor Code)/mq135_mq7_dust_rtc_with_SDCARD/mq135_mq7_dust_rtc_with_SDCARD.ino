/*
  Analog input, analog output, serial output
 
 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.
 
 The circuit:
 * MQ135 connected to analog pin 0.
 * LED connected from digital pin 9 to ground
 
 Copied from sample [AnaloginOutSerial]
 */

#include<SPI.h>
#include<arduino.h>
#include<Wire.h>
#include<SD.h>
#include "RTClib.h"
#include <CS_MQ7.h>

#define MQ7_DEFAULTPPM 0.2 //default ppm of CO for calibration
#define MQ7_RO 10000 //default Ro for MQ7_DEFAULTPPM ppm of CO
#define MQ7_SCALINGFACTOR 9.71 //CO
#define MQ7_EXPONENT 2.5 //CO gas value

CS_MQ7 MQ7(12, 13);  // 12 = digital Pin connected to "tog" from sensor board
                     // 13 = digital Pin connected to LED Power Indicator

#define MQ135_DEFAULTPPM 399 //default ppm of CO2 for calibration
#define MQ135_RO 68550 //default Ro for MQ135_DEFAULTPPM ppm of CO2
#define MQ135_SCALINGFACTOR 116.6020682 //CO2 gas value
#define MQ135_EXPONENT -2.769034857 //CO2 gas value
#define MQ135_MAXRSRO 2.428 //for CO2
#define MQ135_MINRSRO 0.358 //for CO2

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin = A0;  // Analog input pin that the MQ135 is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to
int CoSensorOutput = A1; //analog Pin connected to "out" from sensor board
int CosensorValue = 0;         //analog sensor data
RTC_Millis rtc;
File myFile,myFile1,myFile2;

float NO2;
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int Vcc=5;
int Rl=20000; // 20Kohm Rl

int measurePin = A3;
int ledPower = 7;

int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;
File logfile;

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  pinMode(ledPower,OUTPUT);
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  while(!Serial){
  ;
}
Serial.println("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
   pinMode(10, OUTPUT);
   
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

 }


void loop() {
 myFile = SD.open("test12.csv",FILE_WRITE);
if(myFile){
  DateTime now = rtc.now();
    
    Serial.print(now.year(), DEC);
    Serial.print('/');
    myFile.print(now.year(), DEC);
    myFile.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    myFile.print(now.month(), DEC);
    myFile.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    myFile.print(now.day(), DEC);
    myFile.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    myFile.print(now.hour(), DEC);
    myFile.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    myFile.print(now.minute(), DEC);
    myFile.print(':');
    Serial.print(now.second(), DEC);
    myFile.print(now.second(), DEC);
  // read the analog in value:
 sensorValue = analogRead(analogInPin);            
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);  
  // change the analog out value:
  analogWrite(analogOutPin, outputValue);           

  // print the results to the serial monitor:
  Serial.print("sensor = " );                       
  myFile.print("sensor=");
  Serial.println(sensorValue);      
  myFile.println(sensorValue);

  Serial.print("output = ");      
  myFile.print("output=");
  Serial.println(outputValue); 
  myFile.println(outputValue);   
 
 //for NO2//
 NO2 = map(sensorValue, 0,1023, 0, 100);
  //Serial.print(NO2);
 // Serial.print(";InsertSensorPackageName;MiCS-2710;N02 Gas;NO2;response indicator;RI;0;25;50;75;100");
  //Serial.print("\n");
  Serial.print("NO2 Gas: ");
  myFile.print("NO2 Gas(%)= ");
  Serial.println(NO2);
  myFile.println(NO2);
 //Serial.print("%");
 //myFile.print("%");
 
//for CO2//
long resvalue=((float)22000*(1023-sensorValue)/sensorValue);
Serial.print("ppm value of CO2 = ");
myFile.print("ppm value of CO2 = ");
 Serial.println(mq135_getppm(resvalue,MQ135_RO));
myFile.println(mq135_getppm(resvalue,MQ135_RO));
long mq135_getro(int resvalue,long ppm);

//for Alcohol//
double p=calculatePPM();
Serial.print("Alcohol(in ppm) =");
myFile.print("Alcohol(in ppm) =");
Serial.println(p);
myFile.println(p);
myFile.close();
}
else 
 {// if the file didn't open, print an error:
    Serial.println("error opening testmq135.txt");
 }

  //MQ7
  myFile1 = SD.open("testmq7.csv",FILE_WRITE);
  if(myFile1){
     DateTime now = rtc.now();
    
    Serial.print(now.year(), DEC);
    Serial.print('/');
    myFile1.print(now.year(), DEC);
    myFile1.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    myFile1.print(now.month(), DEC);
    myFile1.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    myFile1.print(now.day(), DEC);
    myFile1.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    myFile1.print(now.hour(), DEC);
    myFile1.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    myFile1.print(now.minute(), DEC);
    myFile1.print(':');
    Serial.print(now.second(), DEC);
    myFile1.print(now.second(), DEC);
   MQ7.CoPwrCycler();  
/* your code here and below! */
   //myFile1 = SD.open("test4.csv",FILE_WRITE);
   //if(myFile1){
  if(MQ7.CurrentState() == LOW){   //we are at 1.4v, read sensor data!
    CosensorValue = analogRead(CoSensorOutput);
    Serial.print("COsensor value");
    myFile1.print("CosensorValue");
    Serial.println(CosensorValue);
    myFile1.println(CosensorValue);
    long resvalue1=((float)22000*(1023-CosensorValue)/CosensorValue);
Serial.print("ppm value of CO = ");
myFile1.print("ppm value of CO = ");
 long ppm=mq7_getppm(resvalue1,MQ7_RO);
 Serial.print(ppm);
   myFile1.println(ppm);
long mq7_getro(int resvalue1,long ppm);
  }
  else{                            //sensor is at 5v, heating time
    Serial.println("sensor heating!");
     
  }      

myFile1.close();
  }
  else 
 {// if the file didn't open, print an error:
    Serial.println("error opening testmq7.txt");
  } 
  //myFile1.close();
  //}
 // else 
 //{// if the file didn't open, print an error:
    //Serial.println("error opening test4.txt");
  //}

//DUST SENSOR
  myFile2 = SD.open("testdust.csv",FILE_WRITE);
 if(myFile2){
  logfile.println("Time, Date, Sensor,Output,NO2 gas,CO2(ppm),Alcohol(ppm)");
  logfile.flush();
   DateTime now = rtc.now();
    
    Serial.print(now.year(), DEC);
    Serial.print('/');
    myFile2.print(now.year(), DEC);
    myFile2.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    myFile2.print(now.month(), DEC);
    myFile2.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    myFile2.print(now.day(), DEC);
    myFile2.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    myFile2.print(now.hour(), DEC);
    myFile2.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    myFile2.print(now.minute(), DEC);
    myFile2.print(':');
    Serial.print(now.second(), DEC);
    myFile2.print(now.second(), DEC);
  digitalWrite(ledPower,LOW); // power on the LED
  delayMicroseconds(samplingTime);

  voMeasured = analogRead(measurePin); // read the dust value
  
  delayMicroseconds(deltaTime);
  digitalWrite(ledPower,HIGH); // turn the LED off
  delayMicroseconds(sleepTime);

  // 0 - 5.0V mapped to 0 - 1023 integer values 
  calcVoltage = voMeasured * (5.0 / 1024); 
  
  // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // Chris Nafis (c) 2012
  dustDensity = (0.17 * calcVoltage - 0.1)*1000; 
  
  Serial.print("Raw Signal Value (0-1023): ");
  myFile2.print("Raw Signal Value (0-1023): ");
  Serial.print(voMeasured);
  myFile2.println(voMeasured);
  Serial.print(" - Voltage: ");
  myFile2.print(" - Voltage: ");
  Serial.println(calcVoltage);
  myFile2.println(calcVoltage);
  Serial.print(" - Dust Density [ug/m3]: ");
  myFile2.print(" - Dust Density [ug/m3]: ");
  Serial.println(dustDensity);
  myFile2.println(dustDensity);
  myFile2.close();
  }
else 
 {// if the file didn't open, print an error:
    Serial.println("error opening testdust.txt");
  }
  // wait 100 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(1000);                     
}


long mq135_getro(long resvalue,long ppm) {
  return (long)(resvalue * exp( log(MQ135_SCALINGFACTOR/ppm) / MQ135_EXPONENT ));
}
double mq135_getppm(long resvalue,long default_RO) 
{
  double ret = 0;
  double validinterval = 0;
  validinterval = resvalue/(double)default_RO;
  if(validinterval<MQ135_MAXRSRO && validinterval>MQ135_MINRSRO) {
    ret = (double)MQ135_SCALINGFACTOR * pow( ((double)resvalue/default_RO), MQ135_EXPONENT);
  }
  return ret;
}

int calculatePPM()
{
  double lgPPM;
  double Vrl=(double)outputValue*Vcc;
  double Rs=Rl*(5-Vrl)/Vrl;
  float ratio=Rs/Rl;
  lgPPM=(log10(ratio)*(-2.6)+2.7);
  double ppm=pow(10,lgPPM);
return ppm;
}

//MQ7 CODE//

 /*long mq7(int CoSensorOutput,int CosensorValue )
 {
   MQ7.CoPwrCycler();  
/* your code here and below! */
  /* myFile1 = SD.open("test4.csv",FILE_WRITE);
   //if(myFile1){
  if(MQ7.CurrentState() == LOW){   //we are at 1.4v, read sensor data!
    CosensorValue = analogRead(CoSensorOutput);
    myFile1.print("CosensorValue");
    Serial.print(CosensorValue);
    myFile1.println(CosensorValue);
    long resvalue=((float)22000*(1023-CosensorValue)/CosensorValue);
Serial.print("ppm value of CO = ");
myFile1.print("ppm value of CO = ");
 long ppm=mq7_getppm(resvalue,MQ7_RO);
 Serial.print(ppm);
   myFile1.println(ppm);
long mq7_getro(int resvalue,long ppm);
  }
  else{                            //sensor is at 5v, heating time
    Serial.println("sensor heating!");
     
  }      

//myFile1.close();
  //}
  //else 
 //{// if the file didn't open, print an error:
   // Serial.println("error opening test4.txt");
  //} 
 }
*/
long mq7_getro(long resvalue1,long ppm) {
  return (long)(resvalue1 * exp( log(MQ7_SCALINGFACTOR/ppm) / MQ7_EXPONENT ));
}
double mq7_getppm(long resvalue1,long default_RO) 
{
  double ret = 0;
  double validinterval = 0;
  validinterval = resvalue1/(double)default_RO;
  //if(validinterval<MQ7_MAXRSRO && validinterval>MQ7_MINRSRO) {
    ret = (double)MQ7_SCALINGFACTOR * pow( ((double)resvalue1/default_RO), MQ7_EXPONENT);
  //}
  return ret;
}
 

