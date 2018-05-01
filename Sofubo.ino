#include <TimerOne.h>
#include <Wire.h>
#include "heartbeat.h"

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}


// Initial time
long int ti;
volatile bool intFlag=false;


void setup() {

  Serial.begin(BAUD_RATE);
  pinMode(INPUT1, INPUT);  

  // Arduino initializations
  Wire.begin();
  Serial.begin(9600);
  
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
 
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  
  
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);
  
   pinMode(13, OUTPUT);
  Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  
  
  // Store initial time
  ti=millis();
}





// Counter
long int cpt=0;

void callback()
{ 
  intFlag=true;
  digitalWrite(13, digitalRead(13) ^ 1);
}


void loop() {

  
  ComputeHeartbeat ();
  computeTemp();
  //DISP_DEL;

  {
  while (!intFlag);
  intFlag=false;
  

  // Read accelerometer and gyroscope
  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);
  
  // Create 16 bits values from 8 bits data

  // Gyroscope
  int16_t gx=-(Buf[8]<<8 | Buf[9]);
  int16_t gy=-(Buf[10]<<8 | Buf[11]);
  int16_t gz=Buf[12]<<8 | Buf[13];
  
  
  // Gyroscope
  Serial.print ("Gyro measurement: ");
  /*Serial.print (gx,DEC); 
  Serial.print ("\t");
  Serial.print (gy,DEC);
  Serial.print ("\t");
  Serial.print (gz,DEC);  
  Serial.print ("\t");*/
  
  if((gx <= 0 && gx >= -20) && (gy <= 20 && gy >= 0) && (gz <= -20 && gz >= -50))
  {
    Serial.print ("The person is steady");
  }
    else{
    Serial.print ("The person is in motion");
  } 
  

  // End of line
  Serial.println("");
  delay(5000);
}
}

void ComputeHeartbeat () {

  int count=0,i=0,k=0,rate=0;
  unsigned long time2,time1;
  unsigned long time;
  
while(k<5)
    {
     if(digitalRead(INPUT1))
     {
      if(k==0)
      time1=millis();
      k++;
      while(digitalRead(INPUT1));
     }
    }
      time2=millis();
      rate=time2-time1;
      rate=rate/5;
      rate=60000/rate;
 
Serial.print("Heart Beat Rate:");
Serial.print(rate);
Serial.println(" ");
}
void computeTemp() {

  //values for temperature
int value = 0;
float volts = 0.0;
float temp = 0.0;

 value = analogRead(A1);
  volts = (value / 1024.0)*5.0;   //conversion to volts
  temp = volts*100.0;             //getting temp celsius

  Serial.print("Temperatue =  ");
  Serial.print(temp);
  Serial.println("  C");
}


  




