#include <Wire.h>

//Angle and Acc
float RateRoll, RatePitch, RateYaw;
float AngleRoll, AnglePitch;
float AccX, AccY, AccZ;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
uint32_t LoopTimer;
//Kalmanfilter parameter
float KalmanAngleRoll = 0;
float KalmanUncertaintyAngleRoll = 2*2;
float KalmanAnglePitch = 0;
float KalmanUncertaintyAnglePitch = 2*2;
float Kalman1DOutput[]={0,0};
//Motor PIN For Left
int m1_A = 12;
int m1_B = 14;
int enA  = 13;
//Motor PIN For Right
int m2_A = 27;
int m2_B = 26;
int enB  = 25;

int buzz = 2;

//Motor PWM set
const int freq = 1000;
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int resolution = 8;
//PID Calculation
int position = 0;
float error = 0;
float error_previous = 0;
float error_integral = 0;
float error_derivative = 0;
float setpoint = -4;
float rightMotor;
float leftMotor;
//PID Gain 
float kp =  28;//52;//78;//75;//55.0;
float ki =  80;//0.00001;//0.0005;
float kd =  3;//7.8;//7.3;//6.5;
float pidCalculate;
//float linear_velocity = 120;
float dt = 0.01;

void gyro_signals(void){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 |
    Wire.read();
  int16_t AccYLSB = Wire.read() << 8 |
    Wire.read();
  int16_t AccZLSB = Wire.read() << 8 |
    Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX = Wire.read() << 8 | 
    Wire.read();
  int16_t GyroY = Wire.read() << 8 |
    Wire.read();
  int16_t GyroZ = Wire.read() << 8 |
    Wire.read();
  RateRoll  = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw   = (float)GyroZ/65.5;

  AccX  = (float)AccXLSB/4096-0.03;
  AccY  = (float)AccYLSB/4096-0.005;
  AccZ  = (float)AccZLSB/4096-0.135;

  AngleRoll  =  atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch = -atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);  
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement){
  KalmanState = KalmanState+0.004*KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004*0.004*4*4;
  float KalmanGain = KalmanUncertainty*1/(1*KalmanUncertainty+3*3);
  KalmanState = KalmanState + KalmanGain*(KalmanMeasurement-KalmanState);
  KalmanUncertainty = (1-KalmanGain)*KalmanUncertainty;
  
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(m1_A, OUTPUT);
  pinMode(m1_B, OUTPUT);
  pinMode(enA, OUTPUT);

  pinMode(m2_A, OUTPUT);
  pinMode(m2_B, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(buzz, OUTPUT);
  
  ledcSetup(pwmChannelA, freq, 8);
  ledcSetup(pwmChannelB, freq, 8);

  ledcAttachPin(enA, pwmChannelA);
  ledcAttachPin(enB, pwmChannelB);

  digitalWrite(m1_A, LOW);
  digitalWrite(m1_B, LOW);
  digitalWrite(m2_A, LOW);
  digitalWrite(m2_B, LOW);
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  for(RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber++){
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    Serial.print("-");
    digitalWrite(buzz, HIGH);
    delay(1);
    digitalWrite(buzz, LOW);
  digitalWrite(buzz, LOW);
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  LoopTimer=micros();
}}

void loop() {
  // put your main code here, to run repeatedly:
  angle_read();
  PID_Calculate();
  //position_hold();
  // Serial.print("Setpoint => ");
  // Serial.print(setpoint);
  // Serial.print("  Current => ");
  // Serial.println(KalmanAngleRoll);
}

void angle_read()
{
  gyro_signals();
  // Serial.print("Acceleration X [g] = ");
  // Serial.print(AccX);
  // Serial.print("  Acceleration Y [g] = ");
  // Serial.print(AccY);
  // Serial.print("  Acceleration Z [g] = ");
  // Serial.println(AccZ);
  // delay(50);
  // Serial.print("Roll Angle = ");
  // Serial.print(AngleRoll);
  // Serial.print("  Pitch Angle = ");
  // Serial.println(AnglePitch);
  // delay(50);
  RateRoll -= RateCalibrationRoll;
  RatePitch-= RateCalibrationPitch;
  RateYaw  -= RateCalibrationYaw;

  kalman_1d(KalmanAngleRoll, 
  KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch,
  KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  position = KalmanAngleRoll; 
   Serial.print("Roll Angle: ");
   Serial.println(KalmanAngleRoll);
  // Serial.print("  Pitch Angle: ");
  // Serial.println(KalmanAnglePitch);
  while(micros()-LoopTimer<4000);
  LoopTimer=micros();
}

void forward()
{
  digitalWrite(m1_A, HIGH);
  digitalWrite(m1_B, LOW);
  ledcWrite(pwmChannelA, rightMotor);
  digitalWrite(m2_A, HIGH);
  digitalWrite(m2_B, LOW);
  ledcWrite(pwmChannelB, leftMotor);
}

void backward()
{
  digitalWrite(m1_A, LOW);
  digitalWrite(m1_B, HIGH);
  ledcWrite(pwmChannelA, -1*rightMotor);
  digitalWrite(m2_A, LOW);
  digitalWrite(m2_B, HIGH);
  ledcWrite(pwmChannelB, -1*leftMotor);
}

void PID_Calculate()
{
  angle_read(); 
  error = KalmanAngleRoll - setpoint;
  error_integral += error*dt;
  error_derivative = (error - error_previous)/dt;
  error_previous = error;
  pidCalculate = kp*error + ki*error_integral + kd*error_derivative;

  if (pidCalculate > 255)  pidCalculate = 255;
  if (pidCalculate < -255) pidCalculate = 255;
 
  if ( KalmanAngleRoll > setpoint )
  {
    digitalWrite(m1_A, LOW);
    digitalWrite(m1_B, HIGH);
    ledcWrite(pwmChannelA, pidCalculate);
    digitalWrite(m2_A, LOW);
    digitalWrite(m2_B, HIGH);
    ledcWrite(pwmChannelB, pidCalculate);
  }

  else if ( KalmanAngleRoll < setpoint )
  {
    digitalWrite(m1_A, HIGH);
    digitalWrite(m1_B, LOW);
    ledcWrite(pwmChannelA, -1*pidCalculate);
    digitalWrite(m2_A, HIGH);
    digitalWrite(m2_B, LOW);
    ledcWrite(pwmChannelB, -1*pidCalculate);
  }
  else 
  {
    //Serial.println("sttop");
    digitalWrite(m1_A, HIGH);
    digitalWrite(m1_B, HIGH);
    ledcWrite(pwmChannelA, 255);
    digitalWrite(m2_A, HIGH);
    digitalWrite(m2_B, HIGH);
    ledcWrite(pwmChannelB, 255);
  }
  
}