#include "HX711.h"
#include "Kalman.h" 
#include <SoftwareSerial.h>
#include <Wire.h>

#define rxPin 4   
#define txPin 3
#define DOUT  5
#define CLK  6                        

SoftwareSerial mySerial(rxPin, txPin);  //RX, TX 
HX711 scale(DOUT, CLK);                 //create a scale instance
Kalman kalmanY;                         //create the Kalman instance

/* I2C Communication with IMU */
const uint8_t IMUAddress = 0x68; 
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication
uint8_t i2cData[14];              //buffer for I2C data

/* IMU */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;    //temperature reading
double kalAngleY;   //calculated angle using a Kalman filter
double compAngleY;  //calculate angle using a complementary filter
double gyroYangle;  //angle calculation using the gyro only

double angle=0.0;     
double compAngle=0.0; 

/* timing */
double dt;  
uint32_t timer;

/* scale */
float calibration_factor = 2625;  //changed sign here 
double F=0.0; //force
double Fold=0.0;

/* other variables */
const double l=0.175;              //length of crank
const double cp=l*2.0;             //constant for power computation "* 2" comes from second crank arm
int count =0;                      //loop counter
double P=0.0;                      //power
double Fsum=0.0;
double w=0.0;
double timer_w=0.0;
uint32_t old_timer_w=0.0;
int indicator=1;


void setup() {
  mySerial.begin(38400);                        //bluetooth device communicates with this baudrate with Arduino 
  //Serial.begin(115200);                       //to output debuging data via USB
  Serial.begin(38400);                          //use this with stamp plot
  scale.set_scale();
  scale.tare();                                 //reset the scale to 0
  long zero_factor = scale.read_average();      //get a baseline reading
  scale.set_scale(calibration_factor);          //adjust to this calibration factor
  initIMU();                                    //initialize IMU
  initKali();                                   //initialize Kalman
  timer = micros(); 
}


void loop() 
{
  kali();
  F=scale.get_units(0);
  
  if(abs(Fold-F)>30){F=Fold;}    //make sure there are no jumps in F because then its likely to be a meassurement fault
  else{Fold=F;}
 
  Fsum=Fsum+F;
  count=count+1;
  
  if(count>400){                //if nothing moves for a too long time 
    w=0.0;
    P=0.0;
  }
  
  if(angle>200&&indicator==0){
    timer_w=double(micros()-old_timer_w)/1000000;     //timer_w in seconds
    old_timer_w=micros();
    w=2*PI/timer_w;
    Fsum=Fsum/count;
    P=cp*Fsum*w;
    count=0;
    Fsum=0.0;
    indicator=1;
  }
  if(angle<100){
  indicator=0;
  }
 
  int wprint=(int) (w*30/PI+0.5);
  int Fprint=(int) (F+0.5);
  int Pprint=(int) (P+0.5);
  int angleprint=(int) (angle+0.5);
    
  send2bl(wprint,Fprint,Pprint,angleprint);
}
  

void send2bl(int w,int F,int P, int a){   
   if(mySerial.available() > 0) 
    {
      int val = mySerial.read();
      if(val==1)
        {if(abs(F)>20){Serial.println("drinnen");}
         mySerial.print(w);
         mySerial.print(";");
         mySerial.print(F);
         mySerial.print(";");
         mySerial.print(P);
         mySerial.print(";");
         mySerial.println(a);
        }  
    }   
}
  
  
void kali(){  
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  dt = (double)(micros() - timer)/ 1000000 ; //calculate delta time
  timer = micros();

  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  double gyroYrate = gyroY / 131.0; //convert to deg/s

 //this fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); //calculate the angle using a Kalman filter

  gyroYangle += gyroYrate * dt;
  //gyroYangle += kalmanY.getRate() * dt; //calculate gyro angle using the unbiased rate
    
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch; //calculate the angle using a complimentary filter
  
  if (gyroYangle < -180 || gyroYangle > 180)  //reset the gyro angle when it has drifted too much
    gyroYangle = kalAngleY;
  
  if(kalAngleY < 0){      //this converts to 0-360 DEG
      angle = 360+kalAngleY;
    }else{
      angle = kalAngleY;
    }
     
     if(compAngleY < 0){      //this converts to 0-360 DEG
      compAngle = 360+compAngleY;
    }else{
      compAngle = compAngleY;
    }
  
  angle=360-angle;      //change direction of pos rotation
}


void initIMU(){
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2;        //set I2C frequency to 400kHz
  i2cData[0] = 7;                             //set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;                          //disable FSYNC and set 260 Hz acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x02;                          //set gyro full scale range to (0:±250deg/s, 1:±500deg/s, 2:±1000deg/s)
  i2cData[3] = 0x00;                          //set accelerometer full scale range to (0:±2g, 1:±4g, 2:±8g)
  while (i2cWrite(0x19, i2cData, 4, false));  //write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true));         //PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    mySerial.print(("Error reading sensor"));
    while (1);
  }
  delay(100);                                 //wait for sensor to stabilize
  while (i2cRead(0x3B, i2cData, 6));          //read register for IMU values
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];
}
 
 
void initKali(){
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  kalmanY.setAngle(pitch);
  gyroYangle = pitch;
  compAngleY = pitch;
} 


uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}


uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode;
}


uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode;
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); 
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; 
      }
    }
  }
  return 0;
}
