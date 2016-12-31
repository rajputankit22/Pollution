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

File myFile,myFile1;

float NO2;
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int Vcc=5;
int Rl=20000; // 20Kohm Rl

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
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
 myFile = SD.open("test3.csv",FILE_WRITE);
 if(myFile){
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
    Serial.println("error opening test3.txt");
  }
  
  myFile1 = SD.open("test4.csv",FILE_WRITE);
  if(myFile1){
  long mq7(int CoSensorOutput,int CosensorValue );
  myFile1.close();
  }
  else 
 {// if the file didn't open, print an error:
    Serial.println("error opening test4.txt");
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

 long mq7(int CoSensorOutput,int CosensorValue )
 {
   MQ7.CoPwrCycler();  
/* your code here and below! */
   myFile1 = SD.open("test4.csv",FILE_WRITE);
   if(myFile1){
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

myFile1.close();
  }
  else 
 {// if the file didn't open, print an error:
    Serial.println("error opening test4.txt");
  } 
 }

long mq7_getro(long resvalue,long ppm) {
  return (long)(resvalue * exp( log(MQ7_SCALINGFACTOR/ppm) / MQ7_EXPONENT ));
}
double mq7_getppm(long resvalue,long default_RO) 
{
  double ret = 0;
  double validinterval = 0;
  validinterval = resvalue/(double)default_RO;
  //if(validinterval<MQ7_MAXRSRO && validinterval>MQ7_MINRSRO) {
    ret = (double)MQ7_SCALINGFACTOR * pow( ((double)resvalue/default_RO), MQ7_EXPONENT);
  //}
  return ret;
}
 
