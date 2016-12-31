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
#include <MySensor.h>  




#define MQ135_DEFAULTPPM 399 //default ppm of CO2 for calibration
#define MQ135_RO 68550 //default Ro for MQ135_DEFAULTPPM ppm of CO2
#define MQ135_SCALINGFACTOR 116.6020682 //CO2 gas value
#define MQ135_EXPONENT -2.769034857 //CO2 gas value
#define MQ135_MAXRSRO 2.428 //for CO2
#define MQ135_MINRSRO 0.358 //for CO2

#define   MQ_SENSOR_ANALOG_PIN         (2)
#define         RL_VALUE                     (5) 
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
#define         GAS_LPG                      (0)
#define         GAS_SMOKE                    (1)
#define         S2SH12_SENSOR                (4)

CS_MQ7 MQ7(12, 13);  // 12 = digital Pin connected to "tog" from sensor board
                     // 13 = digital Pin connected to LED Power Indicator

float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 

float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2:(lg10000,-0.22) 
float           SO2_Curve[2]    =  {40.44109566, -1.085728557};  //2SH12
// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin = A0;  // Analog input pin that the MQ135 is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to
int CoSensorOutput = A1; //analog Pin connected to "out" from sensor board
float Ro = 10000.0;    // this has to be tuned 10K Ohm
int val = 0;
RTC_Millis rtc;
File myFile,myFile1,myFile2,myFile3;

float Ro5 = 2.511;    //2SH12   
float RL5 = 4000;     //2SH12
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


float sensor_volt; 
  float RS_air; //  Get the value of RS via in a clear air
  float RO;  // Get the value of R0 via in H2
  float sensorValue2;
  float valMQ =0.0;
  File logfile; 
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  pinMode(ledPower,OUTPUT);
  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  while(!Serial){
  ;
}
//Serial.println("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin 
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
  // or the SD library functions will not work. 
   pinMode(10, OUTPUT);
   
 //if (!SD.begin(4)) {
   // Serial.println("initialization failed!");
   //return;
  //}
  //Serial.println("initialization done.");
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
Ro5 = MQResistanceCalculation1(analogRead(S2SH12_SENSOR),RL5);
RO =  MQResistanceCalculation(analogRead(A2));
Serial.println("Date \t\tTime \t\tNO2 \tCO2 \tAlcohol \tCO \tDUST \tLPG \tSmoke\tSO2");
 }


void loop() {
 
 /* logfile = SD.open("test139.csv", FILE_WRITE);
   logfile.println("Time, Date, NO2 , CO2 , Alcohol");
  logfile.flush();
 *///myFile = SD.open("1A35.csv",FILE_WRITE);
//if(myFile){

  DateTime now = rtc.now();

    Serial.print(now.year(),DEC);
    Serial.print('/');
    myFile.print(now.year(),DEC);
    myFile.print('/');
    Serial.print(now.month(),DEC);
    Serial.print('/');
    myFile.print(now.month(),DEC);
    myFile.print('/');
    Serial.print(now.day(),DEC);
    Serial.print("@");
    myFile.print(now.day(),DEC);
    myFile.print("");
    Serial.print(now.hour(),DEC);
    Serial.print(':');
    myFile.print(now.hour(),DEC);
    myFile.print(':');
    Serial.print(now.minute(),DEC);
    Serial.print(':');
    myFile.print(now.minute(),DEC);
    myFile.print(':');
    Serial.print(now.second(),DEC);
    myFile.println(now.second(),DEC);
    Serial.print("@");

  // read the analog in value:
 sensorValue = analogRead(analogInPin);            
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);  
  // change the analog out value:
  analogWrite(analogOutPin, outputValue);           

  // print the results to the serial monitor:
//  Serial.print("sensor = " );                       
  //myFile.print("sensor=");
//  Serial.println(sensorValue);      
  //myFile.println(sensorValue);

//  Serial.print("output = ");      
 // myFile.print("output=");
//  Serial.println(outputValue); 
  //myFile.println(outputValue);   
 
 //for NO2//
 NO2 = map(sensorValue, 0,1023, 0, 100);
  //Serial.print(NO2);
 // Serial.print(";InsertSensorPackageName;MiCS-2710;N02 Gas;NO2;response indicator;RI;0;25;50;75;100");
  //Serial.print("\n");
  //Serial.print("NO2 Gas: ");
  //myFile.print("NO2 Gas(%)= ");
  Serial.print(NO2);
  Serial.print("@");
  myFile.print(NO2 );
 //Serial.print("%");
 //myFile.print("%");
 
//for CO2//
long resvalue=((float)22000*(1023-sensorValue)/sensorValue);
//Serial.print("ppm value of CO2 = ");
//myFile.print("ppm value of CO2 = ");
 Serial.print(mq135_getppm(resvalue,MQ135_RO));
 Serial.print("@");
myFile.print(mq135_getppm(resvalue,MQ135_RO) );
long mq135_getro(int resvalue,long ppm);

//for Alcohol//
double p=calculatePPM();
//Serial.print("Alcohol(in ppm) =");
//myFile.print("Alcohol(in ppm) =");
Serial.print(p);
Serial.print("@");
myFile.print("\n");

//myFile.close();
//}
//else 
 //{// if the file didn't open, print an error:
   // Serial.println("error opening testA.txt");
 //}

//MQ7
  //myFile1 = SD.open("7B7.csv",FILE_WRITE);
  //if(myFile1){
     /*DateTime now = rtc.now();
       
    myFile1.print(now.year(), DEC);
    myFile1.print('/');
   
    myFile1.print(now.month(), DEC);
    myFile1.print('/');
   
    myFile1.print(now.day(), DEC);
    myFile1.print(' ');
   
    myFile1.print(now.hour(), DEC);
    myFile1.print(':');
   
    myFile1.print(now.minute(), DEC);
    myFile1.print(':');

    myFile1.print(now.second(), DEC);
     myFile.print(' ');
    */
   MQ7.CoPwrCycler(); 
   
  if(MQ7.CurrentState() == LOW){   //we are at 1.4v, read sensor data!
    val = analogRead(CoSensorOutput);
 float Vrl = val * ( 5.00 / 1024.0  );      // V
  float Rs = 20000 * ( 5.00 - Vrl) / Vrl ;   // Ohm 
  int ratio =  Rs/Ro;            
 // Serial.print ( "CO ppm :");
  Serial.print(get_CO(ratio)); 

   //myFile1 = SD.open("test4.csv",FILE_WRITE);
   //if(myFile1){
  
   
   
//Serial.print("ppm value of CO = ");
myFile1.print("ppm value of CO = ");

 Serial.print("@");
   myFile1.println(get_CO(ratio));

  }
  else{                            //sensor is at 5v, heating time
    Serial.print("SH");
    Serial.print("@");
     
  }      
/*
myFile1.close();
  }
  else 
 {// if the file didn't open, print an error:
    Serial.println("error opening testB.txt");
  } 

  */
 
//DUST SENSOR
 // myFile2 = SD.open("CDC.csv",FILE_WRITE);
 //if(myFile2){
  /* DateTime now = rtc.now();
    
    
    myFile2.print(now.year(), DEC);
    myFile2.print('/');
   
    myFile2.print(now.month(), DEC);
    myFile2.print('/');
   
    myFile2.print(now.day(), DEC);
    myFile2.print(' ');
   
    myFile2.print(now.hour(), DEC);
    myFile2.print(':');
   
    myFile2.print(now.minute(), DEC);
    myFile2.print(':');


    myFile2.print(now.second(), DEC);
     myFile2.print(' ');*/
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
  
 // Serial.print("Raw Signal Value (0-1023): ");
  myFile2.print("Raw Signal Value (0-1023): ");
  //Serial.print(voMeasured);
  myFile2.println(voMeasured);
 // Serial.print(" - Voltage: ");
  myFile2.print(" - Voltage: ");
  //Serial.println(calcVoltage);
  myFile2.println(calcVoltage);
//  Serial.print(" - Dust Density [ug/m3]: ");
  myFile2.print(" - Dust Density [ug/m3]: ");
  Serial.print(dustDensity);
  Serial.print("@");
  myFile2.println(dustDensity);
 // myFile2.close();
  //}
//else 
 //{// if the file didn't open, print an error:
  //  Serial.println("error opening testdust1.txt");
  //}

//MQ-2

  /*--- Get a average data by testing 100 times ---*/   
   for(int x = 0 ; x < 100 ; x++)
  {
    sensorValue2 = sensorValue2 + analogRead(A2);
  }
  sensorValue2 = sensorValue2/100.0;
/*-----------------------------------------------*/
 
  sensor_volt = sensorValue2/1024*5.0;
  RS_air = (5.0-sensor_volt)/sensor_volt; // omit *RL
  RO = RS_air/10.0; // The ratio of RS/R0 is 10 in a clear air

 //myFile3 = SD.open("2D2.csv",FILE_WRITE);
 //if(myFile3){
/*   DateTime now = rtc.now();
    
    
    myFile3.print(now.year(), DEC);
    myFile3.print('/');
   
    myFile3.print(now.month(), DEC);
    myFile3.print('/');
   
    myFile3.print(now.day(), DEC);
    myFile3.print(' ');
   
    myFile3.print(now.hour(), DEC);
    myFile3.print(':');
   
    myFile3.print(now.minute(), DEC);
    myFile3.print(':');


    myFile3.print(now.second(), DEC);
     myFile3.print(' ');*/

  Serial.print(MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/RO,GAS_LPG));
 Serial.print("@");
   //Serial.print( "ppm" );
      myFile3.print(MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/RO,GAS_LPG));
  // Serial.print("SMOKE:"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/RO,GAS_SMOKE));
 //Serial.print("\n");
  Serial.print( "@" );
   myFile3.println(MQGetGasPercentage(MQRead(MQ_SENSOR_ANALOG_PIN)/RO,GAS_SMOKE));
   
 /*myFile3.close();
 }
  else 
 {// if the file didn't open, print an error:
    Serial.println("error opening testmq2.txt");
 }
*/
//2SH12
  
float a=analogRead(S2SH12_SENSOR);
 Serial.print(MQGetGasPercentage1(MQRead1(S2SH12_SENSOR,RL5),Ro5));    
  // wait 100 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  Serial.print("\n");
  delay(5000);                     
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

float get_CO (float ratio){
  float ppm_CO = 0.0;
  ppm_CO = 37143 * pow (ratio, -3.178);
  return ppm_CO;
}

float MQResistanceCalculation1(int raw_adc,float rl_value)
{
  return  (long)((long)(1024*1000*(long)rl_value)/raw_adc-(long)rl_value);

}
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  }  else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
 
  return 0;
}
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}


int MQGetGasPercentage1(float rs_ro_ratio,float ro)
{
  return MQGetPercentage1(rs_ro_ratio,ro,SO2_Curve);
}

int  MQGetPercentage1(float rs_ro_ratio, float ro, float *pcurve)
{
  return (double)(pcurve[0] * pow(((double)rs_ro_ratio/ro), pcurve[1]));
}
float MQRead1(int mq_pin,float rl_value)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation1(analogRead(mq_pin),rl_value);
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}

float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
