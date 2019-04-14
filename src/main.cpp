#include <Arduino.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"



// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D4;
const uint8_t sda = D3;

#define absolut(x)   ( ((x) < 0)?  -(x)  :  (x) )

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
double angle = 0;

double P = 80;
double I = 5;
double D = 5;
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
#define MICROSTEPS 4

//MOTOR 1
#define DIR 5
#define STEP 16
AccelStepper ac1(AccelStepper::DRIVER,STEP,DIR);
// MOTOR 2
#define DIR2 12
#define STEP2 14
AccelStepper ac2(AccelStepper::DRIVER,STEP2,DIR2);

MultiStepper steppers;
////
//// STEPPER TEST
////

/*
#include "DRV8825.h"
//MOTOR 1
#define DIR 5
#define STEP 16
// MOTOR 2
#define DIR2 12
#define STEP2 14
DRV8825 s1(200, DIR, STEP);
DRV8825 s2(200, DIR2, STEP2);
*/


//
// Gyro
//
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


// PID Storage
double uold = 0;
double e[] = {0,0,0};

//
long lastTime = 0;

double controll = 0;

PID pid(&angle, &controll, &soll, P,I,D,DIRECT);

//Prototypen Funktionen
double calcAngle();
void readAccelXZ();
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data);
void initGyro();
void initNetwork();
double calcError(double soll, double ist);
float calcPID(double error);
void initMultiplexer();
void readMultiplexer(double *p, double *i, double *d, double *v);
void initGPIO();

//PID Values
float calcA(double Ts);
float calcB(double Ts);
float calcC(double Ts);

long samplingTime();

void initGPIO()
{
  pinMode(STEP, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(DIR2, OUTPUT);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin(sda, scl);
  Wire.setClock(400000);
  initGPIO();
  initGyro();
  //initNetwork();
  initMultiplexer();
  lastTime = millis();
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(50);
  pid.SetOutputLimits(-1500,1500);

  ac1.setMaxSpeed(2000);
  ac2.setMaxSpeed(2000);

  ac1.setAcceleration(10000);
  ac2.setAcceleration(10000);

  //ac1.setSpeed(1500);
  //ac2.setSpeed(-1500);

  

  //s1.begin(20, 4);
  //s2.begin(20,4);

  //s.setMaxSpeed(500);
  //s.run();
  //stepper.startRotate(65000);  
  /*
  steppers.addStepper(s1);
  steppers.addStepper(s2);*/
  
}

void loop() {
  long time = millis();

  angle = calcAngle();

  pid.Compute();

  ac1.setSpeed(controll);
  ac2.setSpeed(-controll);

  //Serial.print(angle); Serial.print(" "); Serial.print(controll); Serial.print(" P= ");Serial.print(P); Serial.print(" I= ");Serial.print(I); Serial.print(" D="); Serial.println(D);// Serial.println(samplingTime());

  double p = 0;
  double i = 0;
  double d = 0;
  double v = 0;

  readMultiplexer(&p,&i,&d, &v);

  //Serial.println(3*v/1024);
  //Serial.println(v);

  P = 30*p/1024;
  I = 30*i/1024;
  D = d*30/1024;

  pid.SetTunings(P,I,D); 
  ac1.runSpeed();
  ac2.runSpeed();

 while(time + 50>millis())
 {
   ac1.runSpeed();
   ac2.runSpeed();
 }
 
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



float calcPID(double error)
{
  e[2] = e[1];
  e[1] = e[0];
  e[0] = error;
  double Ts = samplingTime();
  double temp = uold + (calcA(Ts)*e[0])+(calcB(Ts)*e[1]) + (calcC(Ts)*e[2]);
  uold = temp;
  return temp;
}

double calcAngle()
{
  fifoCount = mpu.getFIFOCount();

  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetEuler(euler, &q);
  Serial.print("euler\t");
  Serial.print(euler[0] * 180/PI);
  Serial.print("\t");
  Serial.print(euler[1] * 180/PI);
  Serial.print("\t");
  Serial.println(euler[2] * 180/PI);

  
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
{/*
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
  Wire.endTransmission(true);*/

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1688); // 1688 factory default for my test chip

  mpu.setDMPEnabled(true);

}

void initMultiplexer()
{
  // MULTIPLEX
  pinMode(A0, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
}

float calcA(double Ts)
{
  return (P+ (I*Ts/2) + (D/Ts));
}

float calcB(double Ts)
{
  return (-P+(I*Ts/2)-(2*D/Ts));
}

float calcC(double Ts)
{
  return D/Ts;
}

long samplingTime()
{
  long temp = lastTime;
  lastTime = millis();
  return (lastTime - temp);
}

void readMultiplexer(double *p, double *i, double *d, double *v)
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

  //Spannung V
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
  *v = analogRead(A0);
}