#include <SPI.h>
#include <MySensor.h>  
#include <Wire.h> 
#include <CS_MQ7.h>
CS_MQ7 MQ7(12, 13);
int CoSensorOutput = A1; 
float Ro = 10000.0;    // this has to be tuned 10K Ohm
int val = 0;           // variable to store the value coming from the sensor
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 

}

void loop() {
  // put your main code here, to run repeatedly:

  if(MQ7.CurrentState() == LOW){   //we are at 1.4v, read sensor data!
    val = analogRead(CoSensorOutput);
    Serial.print(val);
    Serial.print("\n");
 float Vrl = val * ( 5.00 / 1024.0  );      // V
    Serial.print(Vrl);
    Serial.print("\n");
  float Rs = 20000 * ( 5.00 - Vrl) / Vrl ;   // Ohm 
  Serial.print(Rs);
  Serial.print("\n");
  int ratio =  Rs/Ro;   
  Serial.print(ratio); 
    Serial.print("\n");        
  Serial.print ( "CO ppm :");
  Serial.println(get_CO(ratio));
 
 }
  else{                            //sensor is at 5v, heating time
    Serial.print("SH");
    Serial.print("@");
     
  }      
delay(1000); //delay to allow serial to fully print before sleep
}
float get_CO (float ratio){
  float ppm = 0.0;
  ppm = 37143 * pow (ratio, -3.178);
  return ppm;
}
