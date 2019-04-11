#include <Arduino.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>



// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D4;
const uint8_t sda = D3;

// sensitivity scale factor respective to full scale setting provided in datasheet 
const double AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

//
// Multiplexer
//
const int S0 = D7;      // Multiplex Set Pin
const int S1 = D8;      // Multiplex Set Pin

//
//Network
//
const char *ssid = "Robot_01";
const char *pass = "letmeaccessyourdata";
unsigned int localPort = 2000; // local port to listen for UDP packets
IPAddress ServerIP(192,168,4,1);
IPAddress ClientIP(192,168,4,2);
WiFiUDP udp;
unsigned int localUdpPort = 4210;  // local port to listen on
int16_t AX,AZ;
double ax,az;

//
// PID VALUES
//
double P = 0.01;
double I = 0.01;
double D = 0.01;
double soll = 0;


//
//Motor
//
// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Target RPM for cruise speed
#define RPM 120
// Acceleration and deceleration values are always in FULL steps / s^2
#define MOTOR_ACCEL 2000
#define MOTOR_DECEL 1000

// Microstepping mode. If you hardwired it to save pins, set to the same value here.
#define MICROSTEPS 1

//MOTOR 1
#define DIR 4
#define STEP 16
#include "DRV8825.h"
DRV8825 stepper(MOTOR_STEPS, DIR, STEP);


////
//// STEPPER TEST
////
#include <AccelStepper.h>

AccelStepper s(AccelStepper::DRIVER,14,12);




// PID Storage
double uold = 0;
double e[] = {0,0,0};

//
long lastTime = 0;

//Prototypen Funktionen
double calcAngle();
void readAccelXZ();
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data);
void initGyro();
void initNetwork();
double calcError(double soll, double ist);
double calcPID(double error);
void initMultiplexer();
void readMultiplexer(double *p, double *i, double *d);

//PID Values
double calcA(double Ts);
double calcB(double Ts);
double calcC(double Ts);

double samplingTime();




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(sda, scl);
  initGyro();
  //initNetwork();
  initMultiplexer();
  lastTime = millis();

  s.setMaxSpeed(500);
  //s.run();
  //stepper.startRotate(65000);  
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(calcAngle());
  double angleteest = calcAngle();
  if(angleteest != 0)
    s.setSpeed(300/calcAngle());
  else
    s.setSpeed(0);
  
  s.runSpeed();
/*
  double angle = calcAngle();
  double error = calcError(soll, angle);
  //error = 5;
  double test = calcPID(error);
  Serial.print(error); Serial.print(" "); Serial.println(test);

  double p = 0;
  double i = 0;
  double d = 0;

  readMultiplexer(&p,&i,&d);

  P = p;
  I = i;
  D = d;
/*
  Serial.print("P= "); Serial.print(p);
  Serial.print(" I= "); Serial.print(i);
  Serial.print(" D= "); Serial.println(d);
  /*
  stepper.setRPM(((int)test/abs(test)*60));
  long timer = millis() + abs(test)/10;
  while(timer > millis()){
      stepper.nextAction();
  }
  */
}

void initNetwork()
{
  WiFi.softAP(ssid, pass);    //Create Access point
  //Start UDP
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
}

double calcPID(double error)
{
  e[2] = e[1];
  e[1] = e[0];
  e[0] = error;
  double Ts = samplingTime();
  double temp = uold + calcA(Ts)*e[0]+calcB(Ts)*e[1] + calcC(Ts)*e[2];
  uold = temp;
  return temp;
}

double calcAngle()
{
  
  readAccelXZ();
/*
  double angle = atan(AZ/AX)/(2*PI) * 360;
  angle += (double)8;
  angle *=100;
  if(angle < 0)
  {
    angle = map(angle, -7700, 0, -9000,0);
  }
  else if(angle > 0)
  {
        angle = map(angle, 0, 9500, 0,9000);
  }
  return angle/100;*/

  return (atan(az/ax)* 57.296)+5.0;

}

double calcError(double soll, double ist)
{
  return soll-ist;
}

void readAccelXZ()
{
  Wire.beginTransmission(0x68);
  Wire.write(MPU6050_REGISTER_ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(0x68, (uint8_t)14);

  AX = (((int16_t)Wire.read()<<8) | Wire.read());
  double scrap  = (((int16_t)Wire.read()<<8) | Wire.read());
  AZ = (((int16_t)Wire.read()<<8) | Wire.read());
  scrap  = (((int16_t)Wire.read()<<8) | Wire.read());
  scrap  = (((int16_t)Wire.read()<<8) | Wire.read());
  scrap  = (((int16_t)Wire.read()<<8) | Wire.read());
  scrap  = (((int16_t)Wire.read()<<8) | Wire.read());
  scrap  = (((int16_t)Wire.read()<<8) | Wire.read());
  ax = AX/AccelScaleFactor;
  az = AZ/AccelScaleFactor;

  //Serial.print(*X); Serial.print(" "); Serial.print(*Z); Serial.print(" ");
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void initGyro()
{
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // the config address
  Wire.write(0x06);  // the config value
  Wire.endTransmission(true);
}

void initMultiplexer()
{
  // MULTIPLEX
  pinMode(A0, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
}

double calcA(double Ts)
{
  return (P+ I*Ts/2 + D/Ts);
}

double calcB(double Ts)
{
  return (-P+I*Ts/2-2*D/Ts);
}

double calcC(double Ts)
{
  return D/Ts;
}

double samplingTime()
{
  long temp = lastTime;
  lastTime = millis();
  return lastTime - temp;
}

void readMultiplexer(double *p, double *i, double *d)
{
  //P
  digitalWrite(S0, LOW);
  digitalWrite(S1, LOW);
  *p = analogRead(A0);

  //I
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  *i = analogRead(A0);

  //D
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);
  *d = analogRead(A0);
}