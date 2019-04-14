#include <Arduino.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"



int gyro_address = 0x68;                                     //MPU-6050 I2C address (0x68 or 0x69)
int acc_calibration_value = -64933;//-3;                            //Enter the accelerometer calibration value 1000

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

// sensitivity scale factor respective to full scale setting provided in datasheet 
const double AccelScaleFactor = 16384;
const double GyroScaleFactor = 131;

//Various settings
float pid_p_gain = 15;                                       //Gain setting for the P-controller (15)
float pid_i_gain = 1.5;                                      //Gain setting for the I-controller (1.5)
float pid_d_gain = 30;                                       //Gain setting for the D-controller (30)
float turning_speed = 30;                                    //Turning speed (20)
float max_target_speed = 150;                                //Max target speed (100)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte start, received_byte, low_bat;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int battery_voltage;
int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error_temp, pid_i_mem, pid_setpoint, gyro_input, pid_output, pid_last_d_error;
float pid_output_left, pid_output_right;




// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D4;
const uint8_t sda = D3;


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


void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data);


//MOTOR 1
#define DIR 5
#define STEP 16
// MOTOR 2
#define DIR2 12
#define STEP2 14


void initGyro();
void initNetwork();
void initMultiplexer();
void readMultiplexer(double *p, double *i, double *d, double *v);
void initGPIO();
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress);



//Timer
void initTimer();
void timerCall();

long last;
long delta;

long last2;
long delta2;


void setup() {
  // put your setup code here, to run once:
  noInterrupts();
  Serial.begin(115200);
  Wire.begin(sda, scl);
  Wire.setClock(400000L);
  initTimer();
  initGPIO();
  initGyro();
  //initNetwork();
  initMultiplexer();
  interrupts();
  last = micros();
  last2=micros();

  loop_timer = micros() + 4000;
}

void loop() 
{
  double p = 0;
  double i = 0;
  double d = 0;
  double v = 0;
  readMultiplexer(&p,&i,&d, &v);

  pid_p_gain = 30*p/1024;
  pid_i_gain = 2*i/1024;
  pid_d_gain = 50*d/1024;

  Serial.print("T= ");Serial.print(delta);Serial.print(" P= ");Serial.print(pid_p_gain);Serial.print(" I= ");Serial.print(pid_i_gain);Serial.print(" D= ");Serial.print(pid_d_gain);

  //////////////////////////
  ///// MEIN WINKEL
  //////////////////////////
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  
  Read_RawValue(gyro_address, 0x3B);
  
  //divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor;
  Ay = (double)AccelY/AccelScaleFactor;
  Az = (double)AccelZ/AccelScaleFactor;
  //T = (double)Temperature/340+36.53; //temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;
  angle_gyro = (atan(Az/Ax)* 57.296)+5;


/*
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Angle calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x3F);                                                         //Start reading at register 3F
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 2);                                        //Request 2 bytes from the gyro
  accelerometer_data_raw = Wire.read()<<8|Wire.read();                      //Combine the two bytes to make one integer
  //accelerometer_data_raw /= AccelScaleFactor; //Von mir
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if(accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;           //Prevent division by zero by limiting the acc data to +/-8200;
  if(accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;         //Prevent division by zero by limiting the acc data to +/-8200;

  angle_acc = asin((float)accelerometer_data_raw/8200.0)* 57.296;           //Calculate the current angle according to the accelerometer

  if(start == 0 && angle_acc > -0.5&& angle_acc < 0.5){                     //If the accelerometer angle is almost 0
    angle_gyro = angle_acc;                                                 //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
  }
  
  Wire.beginTransmission(gyro_address);                                     //Start communication with the gyro
  Wire.write(0x43);                                                         //Start reading at register 43
  Wire.endTransmission();                                                   //End the transmission
  Wire.requestFrom(gyro_address, 4);                                        //Request 4 bytes from the gyro
  gyro_yaw_data_raw = Wire.read()<<8|Wire.read();                           //Combine the two bytes to make one integer
  gyro_yaw_data_raw /= GyroScaleFactor;                     
  gyro_pitch_data_raw = Wire.read()<<8|Wire.read();                         //Combine the two bytes to make one integer
  gyro_pitch_data_raw /= GyroScaleFactor;
  
  gyro_pitch_data_raw -= gyro_pitch_calibration_value;                      //Add the gyro calibration value
  angle_gyro += gyro_pitch_data_raw * 0.000031;                             //Calculate the traveled during this loop angle and add this to the angle_gyro variable


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //MPU-6050 offset compensation
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Not every gyro is mounted 100% level with the axis of the robot. This can be cause by misalignments during manufacturing of the breakout board. 
  //As a result the robot will not rotate at the exact same spot and start to make larger and larger circles.
  //To compensate for this behavior a VERY SMALL angle compensation is needed when the robot is rotating.
  //Try 0.0000003 or -0.0000003 first to see if there is any improvement.

  gyro_yaw_data_raw -= gyro_yaw_calibration_value;                          //Add the gyro calibration value
  //Uncomment the following line to make the compensation active
  //angle_gyro -= gyro_yaw_data_raw * 0.0000003;                            //Compensate the gyro offset when the robot is rotating

  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;                    //Correct the drift of the gyro angle with the accelerometer angle
*/
  if(start == 0 && angle_acc > -1&& angle_acc < 1){                         //If the accelerometer angle is almost 0 eigentlich 0.5
                                                                            //Load the accelerometer angle in the angle_gyro variable
    start = 1;                                                              //Set the start variable to start the PID controller
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //PID controller calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The balancing robot is angle driven. First the difference between the desired angel (setpoint) and actual angle (process value)
  //is calculated. The self_balance_pid_setpoint variable is automatically changed to make sure that the robot stays balanced all the time.
  //The (pid_setpoint - pid_output * 0.015) part functions as a brake function.
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  Serial.print(" "); Serial.print(pid_error_temp);
  if(pid_output > 10 || pid_output < -10)pid_error_temp += pid_output * 0.015 ;

  pid_i_mem += pid_i_gain * pid_error_temp;                                 //Calculate the I-controller value and add it to the pid_i_mem variable
  if(pid_i_mem > 400)pid_i_mem = 400;                                       //Limit the I-controller to the maximum controller output
  else if(pid_i_mem < -400)pid_i_mem = -400;
  //Calculate the PID output value
  pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
  if(pid_output > 400)pid_output = 400;                                     //Limit the PI-controller to the maximum controller output
  else if(pid_output < -400)pid_output = -400;

  pid_last_d_error = pid_error_temp;                                        //Store the error for the next loop

  if(pid_output < 5 && pid_output > -5)pid_output = 0;                      //Create a dead-band to stop the motors when the robot is balanced

  if(angle_gyro > 30 || angle_gyro < -30 || start == 0 || low_bat == 1){    //If the robot tips over or the start variable is zero or the battery is empty
    pid_output = 0;                                                         //Set the PID controller output to 0 so the motors stop moving
    pid_i_mem = 0;                                                          //Reset the I-controller memory
    start = 0;                                                              //Set the start variable to 0
    self_balance_pid_setpoint = 0;                                          //Reset the self_balance_pid_setpoint variable
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Control calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.print(" ");Serial.println(pid_output);// Serial.print(" "); Serial.print(delta);Serial.print(" "); Serial.println(delta2);
  pid_output_left = pid_output;                                             //Copy the controller output to the pid_output_left variable for the left motor
  pid_output_right = pid_output;                                            //Copy the controller output to the pid_output_right variable for the right motor
/*    LENKUNG KOMMT SPÄTER
  if(received_byte & B00000001){                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    pid_output_left += turning_speed;                                       //Increase the left motor speed
    pid_output_right -= turning_speed;                                      //Decrease the right motor speed
  }
  if(received_byte & B00000010){                                            //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    pid_output_right += turning_speed;                                      //Increase the right motor speed
  }

  if(received_byte & B00000100){                                            //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint > -2.5)pid_setpoint -= 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forewards
    if(pid_output > max_target_speed * -1)pid_setpoint -= 0.005;            //Slowly change the setpoint angle so the robot starts leaning forewards
  }
  if(received_byte & B00001000){                                            //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    if(pid_setpoint < 2.5)pid_setpoint += 0.05;                             //Slowly change the setpoint angle so the robot starts leaning backwards
    if(pid_output < max_target_speed)pid_setpoint += 0.005;                 //Slowly change the setpoint angle so the robot starts leaning backwards
  }   

  if(!(received_byte & B00001100)){                                         //Slowly reduce the setpoint to zero if no foreward or backward command is given
    if(pid_setpoint > 0.5)pid_setpoint -=0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if(pid_setpoint < -0.5)pid_setpoint +=0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else pid_setpoint = 0;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }
*/  
  //The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if(pid_setpoint == 0){                                                    //If the setpoint is zero degrees
    if(pid_output < 0)self_balance_pid_setpoint += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if(pid_output > 0)self_balance_pid_setpoint -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Motor pulse calculations
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //To compensate for the non-linear behaviour of the stepper motors the folowing calculations are needed to get a linear speed behaviour.
  if(pid_output_left > 0)pid_output_left = 405 - (1/(pid_output_left + 9)) * 5500;
  else if(pid_output_left < 0)pid_output_left = -405 - (1/(pid_output_left - 9)) * 5500;

  if(pid_output_right > 0)pid_output_right = 405 - (1/(pid_output_right + 9)) * 5500;
  else if(pid_output_right < 0)pid_output_right = -405 - (1/(pid_output_right - 9)) * 5500;

  //Calculate the needed pulse time for the left and right stepper motor controllers
  if(pid_output_left > 0)left_motor = 400 - pid_output_left;
  else if(pid_output_left < 0)left_motor = -400 - pid_output_left;
  else left_motor = 0;

  if(pid_output_right > 0)right_motor = 400 - pid_output_right;
  else if(pid_output_right < 0)right_motor = -400 - pid_output_right;
  else right_motor = 0;

  //Copy the pulse time to the throttle variables so the interrupt subroutine can use them
  throttle_left_motor = left_motor/2; // /2 weil nur halbsoviele Halbschritte wie Viertelschritte nötig sind
  throttle_right_motor = right_motor/2;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Loop time timer
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //The angle calculations are tuned for a loop time of 4 milliseconds. To make sure every loop is exactly 4 milliseconds a wait loop
  //is created by setting the loop_timer variable to +4000 microseconds every loop.
  while(loop_timer > micros())
  {
    //delay(0);
  }
  loop_timer += 4000;
  double temp = micros();
  delta2 = temp-last2;
  last2 = temp;
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

void initTimer()
{
  timer1_isr_init();
  timer1_attachInterrupt(timerCall);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP); // 5 ticks/us -> 100 ticks für 20us
  //timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
  timer1_write(200);
  //timer1_write(ESP.getCycleCount()+1600L);
}

void ICACHE_RAM_ATTR timerCall()
{
  throttle_counter_left_motor ++;                                           //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if(throttle_counter_left_motor > throttle_left_motor_memory){             //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if(throttle_left_motor_memory < 0){                                     //If the throttle_left_motor_memory is negative
      digitalWrite(DIR, HIGH);                                               //Set output 3 low to reverse the direction of the stepper controller
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else digitalWrite(DIR, LOW);                                           //Set output 3 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_left_motor == 1)digitalWrite(STEP, HIGH);        //Set output 2 high to create a pulse for the stepper controller
  else if(throttle_counter_left_motor == 2)digitalWrite(STEP, LOW);         //Set output 2 low because the pulse only has to last for 20us 
  
  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(throttle_counter_right_motor > throttle_right_motor_memory){           //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if(throttle_right_motor_memory < 0){                                    //If the throttle_right_motor_memory is negative
      digitalWrite(DIR2, LOW);                                              //Set output 5 low to reverse the direction of the stepper controller
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else digitalWrite(DIR2, HIGH);                                          //Set output 5 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_right_motor == 1)digitalWrite(STEP2, HIGH);      //Set output 4 high to create a pulse for the stepper controller
  else if(throttle_counter_right_motor == 2)digitalWrite(STEP2, LOW);       //Set output 4 low because the pulse only has to last for 20us

  long temp = micros();
  delta = temp - last;
  last = temp;
}

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

void initGyro()
{/*
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6B);                                                         //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x6C);                                                         //We want to write to the PWR_MGMT_2 register (6B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                   //End the transmission with the gyro.
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1B);                                                         //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search.
  Wire.write(0x1C);                                                         //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x08);                                                         //Set the register bits as 00001000 (+/- 4g full scale range)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(gyro_address);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();

  for(receive_counter = 0; receive_counter < 500; receive_counter++){       //Create 500 loops
    //if(receive_counter % 15 == 0)digitalWrite(13, !digitalRead(13));        //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro
    Wire.write(0x43);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(gyro_address, 4);                                      //Request 2 bytes from the gyro
    gyro_yaw_calibration_value += Wire.read()<<8|Wire.read();               //Combine the two bytes to make one integer
    gyro_pitch_calibration_value += Wire.read()<<8|Wire.read();             //Combine the two bytes to make one integer
    delayMicroseconds(3700);                                                //Wait for 3700 microseconds to simulate the main program loop time
  }
  gyro_pitch_calibration_value /= 500;                                      //Divide the total value by 500 to get the avarage gyro offset
  gyro_yaw_calibration_value /= 500;                                        //Divide the total value by 500 to get the avarage gyro offset
  Serial.println("Gyro kalibriert");*/

  I2C_Write(gyro_address, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(gyro_address, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(gyro_address, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(gyro_address, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(gyro_address, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(gyro_address, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(gyro_address, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(gyro_address, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(gyro_address, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(gyro_address, MPU6050_REGISTER_USER_CTRL, 0x00);

}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);  // the config address
  Wire.write(0x06);  // the config value
  Wire.endTransmission(true);

  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

void initMultiplexer()
{
  // MULTIPLEX
  pinMode(A0, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
}

void initGPIO()
{
  pinMode(STEP, OUTPUT);
  pinMode(STEP2, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(DIR2, OUTPUT);
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
/*
  //Spannung V
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
  *v = analogRead(A0);*/
}