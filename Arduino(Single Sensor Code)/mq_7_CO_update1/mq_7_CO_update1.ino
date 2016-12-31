/* 
 Citizen Sensor
 http://citizensensor.cc
 MQ-7 Carbon Monoxide Sensor
 Power Cycle + Analog Read
 
 for this example:
 - the "tog" pin of the breakout should be connected to digital pin 12.
 - the "out" pin of the breakout should be connected to analog pin 0.
 
 >> When the sensor is receiving 5v of power, the LED Indicator (Pin 13) will
 >> be ON.  During this time data the data being output is NOT readable.
 >> When the sensor is receiving 1.4v or power, the LED Indicator will be
 >> OFF.  This when the data being output is USABLE. 
 
 */

#include <CS_MQ7.h>

#define MQ7_DEFAULTPPM 0.2 //default ppm of CO for calibration
#define MQ7_RO 10000 //default Ro for MQ7_DEFAULTPPM ppm of CO
#define MQ7_SCALINGFACTOR 9.71 //CO
#define MQ7_EXPONENT 2.5 //CO gas value

CS_MQ7 MQ7(12, 13);  // 12 = digital Pin connected to "tog" from sensor board
                     // 13 = digital Pin connected to LED Power Indicator

int CoSensorOutput = A0; //analog Pin connected to "out" from sensor board
int sensorValue = 0;         //analog sensor data

void setup(){
  
  Serial.begin(9600);
}

void loop(){

  MQ7.CoPwrCycler();  


  /* your code here and below! */
  
  if(MQ7.CurrentState() == LOW){   //we are at 1.4v, read sensor data!
    sensorValue = analogRead(CoSensorOutput);
    Serial.println(sensorValue);
    long resvalue=((float)22000*(1023-sensorValue)/sensorValue);
Serial.print("ppm value of CO = ");
 long ppm=mq7_getppm(resvalue,MQ7_RO);
 Serial.print(ppm);
long mq7_getro(int resvalue,long ppm);
  }
  else{                            //sensor is at 5v, heating time
    Serial.println("sensor heating!");
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
