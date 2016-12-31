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
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Wire.h>
int j=1;
uint8_t i = 0;

const int chipSelect = 4;
char gpsfilename[] = "Gy8LOG00/GPSLOG.txt";


char foldername[]= "Gy8LOG00";
File gpsfile;
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true
#define LOG_FIXONLY false  
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = true;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
// Set the pins used
#define chipSelect 4
#define ledPin 13

File logfile;

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A')+10;
}

// blink out an error code
void error(uint8_t errno) {
   while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}















































#include<SPI.h>
#include<arduino.h>
#include<Wire.h>
#include<SD.h>
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

File myFile;

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
    Serial.println("error opening test.txt");
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
