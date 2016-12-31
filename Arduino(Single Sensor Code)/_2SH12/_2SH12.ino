#include <SPI.h>
#include <MySensor.h>  
#include <Wire.h> 
#define         S2SH12_SENSOR                (15)

#define         CALIBRATION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 

//#define         GAS_SO2                     (17) 

float           SO2_Curve[2]    =  {40.44109566, -1.085728557};  //MQ136
float           Ro              =  10000;                          //Ro is initialized to 10 kilo ohms

float Ro5 = 2.511;    //2SH12   
float RL5 = 4000;     //2SH12   MQ-XL-V2 auto-ctrl.com 

int val = 0;          // variable to store the value coming from the sensor

void setup() {
  // put your setup code here, to run once:
   Serial.begin(9600); 
   Serial.print("    2SH12:"); 
  Ro5 = MQResistanceCalculation(analogRead(S2SH12_SENSOR),RL5);
  //Serial.println(Ro5);

}

void loop() {
  // put your main code here, to run repeatedly:
 //2SH12
   Serial.print("2SH12  :"); 
   Serial.print("SO2   :"); 
   float a=analogRead(S2SH12_SENSOR);
   Serial.print(a);   
   Serial.print( " raw " ); 
    Serial.print(MQGetGasPercentage(MQRead(S2SH12_SENSOR,RL5),Ro5));     
   Serial.print("\n");  
   delay(1000);
}
float MQResistanceCalculation(int raw_adc,float rl_value)
{
//  return ( ((float)rl_value*(1023-raw_adc)/raw_adc));
  return  (long)((long)(1024*1000*(long)rl_value)/raw_adc-(long)rl_value);

}

int MQGetGasPercentage(float rs_ro_ratio,float ro)
{
  return MQGetPercentage(rs_ro_ratio,ro,SO2_Curve);
}

int  MQGetPercentage(float rs_ro_ratio, float ro, float *pcurve)
{
  return (double)(pcurve[0] * pow(((double)rs_ro_ratio/ro), pcurve[1]));
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
float MQCalibration(int mq_pin, double ppm, double rl_value,float *pcurve )
{
  int i;
  float val=0;

  for (i=0;i<CALIBRATION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin),rl_value);
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBRATION_SAMPLE_TIMES;                   //calculate the average value
  //Ro = Rs * sqrt(a/ppm, b) = Rs * exp( ln(a/ppm) / b )

  return  (long)val*exp((log(pcurve[0]/ppm)/pcurve[1]));

}
