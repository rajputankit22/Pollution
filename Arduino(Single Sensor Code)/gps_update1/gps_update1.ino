#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <Wire.h>

//#include <avr/sleep.h>


/*#include <SoftwareSerial.h>


#include <SD.h>
#include <Adafruit_BMP085.h>
#include <L3G4200D.h>
#define Register_ID 0
#define Register_2D 0x2D                      
#define Register_X0 0x32
#define Register_X1 0x33
#define Register_Y0 0x34
#define Register_Y1 0x35
#define Register_Z0 0x36
#define Register_Z1 0x37

int ADXAddress = 0xA7 >> 1;  // the default 7-bit slave address
int reading = 0;
int val=0;
int X0,X1,X_out;
int Y0,Y1,Y_out;
int Z1,Z0,Z_out;
double Xg,Yg,Zg;

#define address 0x1E//I2C address of HMC5883
#define BMP085_ADDRESS 0x77  // I2C address of BMP085

*/int j=1;
uint8_t i = 0;

const int chipSelect = 4;
/*char afilename[] = "Gy8LOG00/AccLOG.csv";
char mfilename[] = "Gy8LOG00/MagLOG.csv";
char tfilename[] = "Gy8LOG00/TemLOG.csv";
char gfilename[] = "Gy8LOG00/GyrLOG.csv";
*/char gpsfilename[] = "Gy8LOG00/GPSLOG.txt";


char foldername[]= "Gy8LOG00";

/*File afile;
File mfile;
File tfile;
File gfile;
*/File gpsfile;


/*Adafruit_BMP085 bmp;
L3G4200D gyro;
// Ladyada's logger modified by Bill Greiman to use the SdFat library
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS Shield
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

 */HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true
/* set to true to only log to SD when GPS has a fix, for debugging, keep it false */
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
  /*
  if (SD.errorCode()) {
   putstring("SD error: ");
   Serial.print(card.errorCode(), HEX);
   Serial.print(',');
   Serial.println(card.errorData(), HEX);
   }
   */
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

void setup() {
  // for Leonardos, if you want to debug SD issues, uncomment this line
  // to see serial output
  //while (!Serial);

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  
  Wire.begin();                
  Serial.begin(9600);    
  delay(100);
  // enable to measute g data
  /*Wire.beginTransmission(ADXAddress);
  Wire.write(Register_2D);
  Wire.write(8);                //measuring enable
  Wire.endTransmission(); 
  */Serial.begin(115200);
  Serial.println("\r\nUltimate GPSlogger Shield");
  pinMode(ledPin, OUTPUT);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    //if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
    Serial.println("Card init. failed!");
    error(2);
  }
  char filename[15];
  strcpy(filename, "GPSLOG00.CSV");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error(3);
  }
  Serial.print("Writing to "); 
  Serial.println(filename);
  logfile.println("Time, Date, Latitude, Longitude, Elevation, Speed (Knots), Angle, Satellites");
  logfile.flush();



  // connect to the GPS at the desired rate
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For logging data, we don't suggest using anything but either RMC only or RMC+GGA
  // to keep the log files at a reasonable size
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 or 5 Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
  
  /*for (i = 0; i < 100; i++) {
    afilename[6] = i/10 + '0';
    afilename[7] = i%10 + '0';
    if (! SD.exists(afilename)) {
       foldername[6] = i/10 + '0';
       foldername[7] = i%10 + '0';
      SD.mkdir(foldername);
      // only open a new file if it doesn't exist
      afile = SD.open(afilename, FILE_WRITE);
      afile.println("Seconds-interval,X,Y,Z");
      afile.close();
      
       
      break;  // leave the loop!
    }
   
}
    mfilename[6] = i/10 + '0';
    mfilename[7] = i%10 + '0';
    
     tfilename[6] = i/10 + '0';
    tfilename[7] = i%10 + '0';
    
    gfilename[6] = i/10 + '0';
    gfilename[7] = i%10 + '0';
    
    */gpsfilename[6] = i/10 + '0';
    gpsfilename[7] = i%10 + '0';
    /*
      mfile = SD.open(mfilename, FILE_WRITE);
      mfile.println("Seconds,X,Y,Z");
      mfile.close();
      
      
      tfile = SD.open(tfilename, FILE_WRITE);
      tfile.println("Seconds-interval,Temp,Pressure,Altitude,Sea-Level Pressure");
      tfile.close();
      
      gfile = SD.open(gfilename, FILE_WRITE);
      gfile.println("Seconds-interval,X,Y,Z");
      gfile.close();
      
     */ gpsfile = SD.open(gpsfilename, FILE_WRITE);
      //gpsfile.println("Time, Date, Latitude, Longitude, Elevation, Speed (Knots), Angle, Satellites");
     
      gpsfile.close();
      
      
    GPS.begin(9600);

      
     // gyro.enableDefault();
      

  Serial.println("Ready!");
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c;
  while (mySerial.available())
  {
    c = GPS.read();
    // if you want to debug, this is a good time to do it!
#ifdef UDR0
    if (GPSECHO)
      if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
  }
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop() {
  gpsfile = SD.open(gpsfilename, FILE_WRITE);
  char c;
  while (mySerial.available())
  {
    c = GPS.read();
    if (GPSECHO)
      if (c)   Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another

    // Sentence parsed! 
    Serial.println("OK");
    if (LOG_FIXONLY && !GPS.fix) {
      Serial.print("No Fix");
      return;
    }

    // Rad. lets log it!
    Serial.println("Log");

      char *stringptr = GPS.lastNMEA();
    uint8_t stringsize = strlen(stringptr);
    if (stringsize != gpsfile.write((uint8_t *)stringptr, stringsize))    //write the string to the SD file
        error(4);
    if (strstr(stringptr, "RMC"))   gpsfile.flush();
    Serial.println();
     gpsfile.close();
      Serial.println("success."); 
    Serial.println();
    logfile.print(GPS.hour, DEC);
      logfile.print(':');
      logfile.print(GPS.minute, DEC);
      logfile.print(':');
      logfile.print(GPS.seconds, DEC);
      logfile.print('.');
      logfile.print(GPS.milliseconds);
      logfile.print(",");

      logfile.print(GPS.month, DEC); 
      logfile.print('/');
      logfile.print(GPS.day, DEC);
      logfile.print("/20");
      logfile.print(GPS.year, DEC);
      logfile.print(",");

      logfile.print(GPS.latitude, 4);
      logfile.print(GPS.lat);
      logfile.print(", ");
      logfile.print(GPS.longitude, 4);
      logfile.print(GPS.lon);
      logfile.print(",");
      logfile.print(GPS.altitude);
      logfile.print(",");
      logfile.print(GPS.speed);
      logfile.print(",");
      logfile.print(GPS.angle);
      logfile.print(",");
      logfile.println((int)GPS.satellites);
      logfile.flush();
      Serial.println("success.");
  }
 /* Wire.begin();
    Wire.beginTransmission(ADXAddress); // transmit to device
  Wire.write(Register_X0);
  Wire.write(Register_X1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress,2); 
  if(Wire.available()<=2)   
  {
    X0 = Wire.read();
    X1 = Wire.read(); 
    X1=X1<<8;
    X_out=X0+X1;   
  }

  //------------------Y
  Wire.beginTransmission(ADXAddress); // transmit to device
  Wire.write(Register_Y0);
  Wire.write(Register_Y1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress,2); 
  if(Wire.available()<=2)   
  {
    Y0 = Wire.read();
    Y1 = Wire.read(); 
    Y1=Y1<<8;
    Y_out=Y0+Y1;
  }
  //------------------Z
  Wire.beginTransmission(ADXAddress); // transmit to device
  Wire.write(Register_Z0);
  Wire.write(Register_Z1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress,2); 
  if(Wire.available()<=2)   
  {
    Z0 = Wire.read();
    Z1 = Wire.read(); 
    Z1=Z1<<8;
    Z_out=Z0+Z1;
  }
  //
  Xg=X_out/256.0;
  Yg=Y_out/256.0;
  Zg=Z_out/256.0;
  Serial.print("X= ");
  Serial.print(Xg);
  Serial.print("       ");
  Serial.print("Y= ");
  Serial.print(Yg);
  Serial.print("       ");
  Serial.print("Z= ");
  Serial.print(Zg);
  Serial.println("  ");
  delay(200);
  //File dataFile = SD.open(afilename, FILE_WRITE);
  afile = SD.open(afilename, FILE_WRITE); 
    
  //afile.print(j);
  //afile.print(",");
  afile.print(millis()/1000.0);
  afile.print(",");
      
  afile.print(Xg);
  afile.print(",");
 
  afile.print(Yg);
  afile.print(",");
  
  afile.print(Zg);
  afile.println("");
   afile.close();
   
   int x,y,z; //triple axis data

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  mfile = SD.open(mfilename, FILE_WRITE);
  //Print out values of each axis
  mfile.print(millis()/1000.0);
  mfile.print(",");
  mfile.print(x);
  mfile.print(",");
  mfile.print(y);
  mfile.print(",");
  mfile.println(z);
  mfile.close();
  
   bmp.begin();
    tfile = SD.open(tfilename, FILE_WRITE);
    tfile.print(millis()/1000.0);
  tfile.print(",");
   tfile.print(bmp.readTemperature());
   
   tfile.print(",");
   
   tfile.print(bmp.readPressure());
    tfile.print(",");
    
    tfile.print(bmp.readAltitude());
    tfile.print(",");
    
    tfile.println(bmp.readSealevelPressure());
    tfile.close();
    
    gyro.read();
gfile = SD.open(gfilename, FILE_WRITE);
  gfile.print(millis()/1000.0);
  gfile.print(",");
  gfile.print((int)gyro.g.x);
  gfile.print(",");
  gfile.print((int)gyro.g.y);
  gfile.print(",");
  gfile.println((int)gyro.g.z);
  gfile.close();
 */ 
}



/* End code */

