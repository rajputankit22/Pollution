#include <SPI.h>
#include <MySensor.h>  
#include <Wire.h> 
#define         MQ2_SENSOR                   (2)  //define which analog input channel you are going to use
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation

#define         GAS_LPG                     (1)
#define         GAS_Smoke                   (2)
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
float           LPGCurve[2]     =  {591.6128784, -1.679699732};  //MQ2
float           SmokeCurve[2]   =  {3426.376355, -2.225037973};  //MQ2
float Ro0 = 4.300;    //MQ2     3.83 this has to be tuned 10K Ohm
float RL0 = 2.897;    //MQ2     Elecfreacks Octopus
void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
Ro0=MQResistanceCalculation(analogRead(MQ2_SENSOR),RL0);
}

void loop() {
  // put your main code here, to run repeatedly:
 Serial.print("MQ2    :"); 
   Serial.print("LPG   :"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ2_SENSOR,RL0),Ro0,GAS_LPG) );
   Serial.print( "ppm" );
   Serial.print("    ");   
   
   Serial.print("SMOKE :"); 
   Serial.print(MQGetGasPercentage(MQRead(MQ2_SENSOR,RL0),Ro0,GAS_Smoke) );
              Serial.print( "ppm" );
   Serial.print("\n");  
}
float MQRead(int mq_pin,float rl_value)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin),rl_value);
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 int MQGetGasPercentage(float rs_ro_ratio, float ro, int gas_id)
{
   if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,ro,LPGCurve);     //MQ2
    } else if ( gas_id == GAS_Smoke ) {
     return MQGetPercentage(rs_ro_ratio,ro,SmokeCurve);   //MQ2
    } 
}
int  MQGetPercentage(float rs_ro_ratio, float ro, float *pcurve)
{
  return (double)(pcurve[0] * pow(((double)rs_ro_ratio/ro), pcurve[1]));
}
float MQResistanceCalculation(int raw_adc,float rl_value)
{
//  return ( ((float)rl_value*(1023-raw_adc)/raw_adc));
  return  (long)((long)(1024*1000*(long)rl_value)/raw_adc-(long)rl_value);

}
