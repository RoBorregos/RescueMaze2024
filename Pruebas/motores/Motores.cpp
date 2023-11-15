#include "Motores.h"
#include "Encoder.h"

Motor::Motor(uint8_t digital_one, uint8_t digital_two /*,int pid_straight_(kPStraight, kIStraight, kDStraight, kPidMinMinOutputLimit, kPidMaxOutputLimit, kPidMaxErrorSum, kPidMotorTimeSample),
pid_rotate_(kPRotate, kIRotate, kDRotate, kPidMinOutputLimit, kPidMaxOutputLimit, kPidMaxErrorSum, kPidMotorTimeSample*/) {
    digital_one_ = digitalOne;
    digital_two_ = digitalTwo;  
    pwm_pin_ = pwmPin;
    motorID_ = motorID;
    encoderA_ = encoderA;
    encoderB_ = encoderB;
}
//Getters Definidos
uint8_t Motor::getEncoderA(){
  return encoderA_;
}
  
uint8_t Motor::getEncoderB(){
  return encoderB_;
}

MotorState Motor::getCurrentState(){
  return current_state_;
}
  
int Motor::getEncoderTics(){
  return tics_counter_;
}
  
double Motor::getCurrentSpeed(){
  return current_speed_;
}
  
double Motor::getTargetSpeed(){
  return RPM2RPS(target_speed_);
}

int Motor::getPidTics(){
  return pid_tics_;
}
  

//Metodos Definidos
void Motor::motorSetup(){
  pinMode(digital_one_, OUTPUT);
  pinMode(digital_two_, OUTPUT);
  pinMode(pwm_pin_, OUTPUT);
  pinMode(encoderA_, INPUT);
  pinMode(encoderB_, INPUT);
}

void Motor::initEncoders(){ 
  switch(motorID_){
    case MotorID::backLeft:
      attachInterrupt(digitalPinToInterrupt(encoderA_), Encoder::backLeftEncoder, RISING);
    break;
    case MotorID::frontLeft:
      attachInterrupt(digitalPinToInterrupt(encoderA_), Encoder::frontLeftEncoder, RISING);
    break;
    case MotorID::backRight:
      attachInterrupt(digitalPinToInterrupt(encoderA_), Encoder::backRightEncoder, RISING);
    break;
    case MotorID::frontRight:
      attachInterrupt(digitalPinToInterrupt(encoderA_), Encoder::frontRightEncoder, RISING);
    break;
  }
}

void Motor::deltaEncoderTics(int delta){
  tics_counter_ += delta;
}

void Motor::deltaPidTics(int delta){
  pid_tics_ += delta; 
}

//Control
void Motor::motorForward(){
  analogWrite(pwm_pin_, pwm_);
  if (current_state_ == MotorState::Forward){
    return;
  }

  
  digitalWrite(digital_one_, HIGH);
  digitalWrite(digital_two_, LOW);
  
  pid_straight_.reset();
  pid_rotate_.reset();

  current_state_ = MotorState::Forward;
}

void Motor::motorBackward(){
  analogWrite(pwm_pin_, pwm_);

  if (current_state_ == MotorState::Backward){
    return;
  }
  
  digitalWrite(digital_one_, LOW);
  digitalWrite(digital_two_, HIGH);

  pid_straight_.reset();
  pid_rotate_.reset();

  current_state_ = MotorState::Backward;
}
    
void Motor::motorStop(){
  analogWrite(pwm_pin_, LOW);

  if (current_state_ == MotorState::Stop){
    return;
  }
  
  digitalWrite(digital_one_, LOW);
  digitalWrite(digital_two_, LOW);

  pid_straight_.reset();
  pid_rotate_.reset();

  current_state_ = MotorState::Stop;
}

// Velocity. 
void Motor::setPWM(double PWM){
  pwm_ = PWM;
  switch(current_state_) {
    case MotorState::Forward:
      motorForward();
    break;
    case MotorState::Backward:
      motorBackward();
    break;
    case MotorState::Stop:
      motorStop();
    break;
  }
}

double Motor::getTargetRps(double velocity){
  return Ms2Rps(velocity);
}

double Motor::RPM2RPS(double velocity){
  return velocity/kSecondsInMinute;
}

double Motor::Ms2Rps(double MS){
  return (MS / (M_PI * kWheelDiameter));
}

double Motor::Pwm2Rpm(double pwm){
  return ((pwm * kRPM) / kMaxPWM);
}

void Motor::motorSpeedPID(double target_speed){
  int speed_sign = min(1, max(-1, target_speed * 1000));
  target_speed_ = fabs(target_speed);
  double tmp_pwm = pwm_;
  switch (speed_sign)
  {
  case 0:
    motorStop();
  break;
  case 1:
    motorForward();
  break;
  case -1:
    motorBackward();
  break;
  }
  
  pid_straight_.computeSpeed(RPM2RPS(target_speed_), current_speed_, tmp_pwm, pid_tics_, kPulsesPerRevolution, kPidCountTimeSamplesInOneSecond);
  
  setPWM(tmp_pwm);
}

void Motor::motorRotateIzqPID(double target_angle, double current_angle){
  double tmp_pwm = pwm_;
  pid_rotate_.computeRotateIzq(target_angle, current_angle, tmp_pwm);
  tmp_pwm = fabs(tmp_pwm);
  if (tmp_pwm < 70){
    tmp_pwm = 70;
  } else if (tmp_pwm > 255){
    tmp_pwm = 255;
  }
  setPWM(tmp_pwm);
}

void Motor::motorRotateDerPID(double target_angle, double current_angle){
  double tmp_pwm = pwm_;
  pid_rotate_.computeRotateDer(target_angle, current_angle, tmp_pwm);
  tmp_pwm = fabs(tmp_pwm);
  if (tmp_pwm < 70){
    tmp_pwm = 70;
  } else if (tmp_pwm > 255){
    tmp_pwm = 255;
  }
  setPWM(tmp_pwm);
}

double Motor::getPWM(){
  return pwm_;
}

void Motor::motorSpeedPWM(double target_speed){
  int speed_sign = min(1, max(-1, target_speed * 1000));
  target_speed_ = fabs(target_speed);
  pwm_ = target_speed_;

  switch (speed_sign)
  {
  case 0:
    motorStop();
  break;
  case 1:
    motorForward();
  break;
  case -1:
    motorBackward();
  break;
  }
}

double Motor::getDistanceTraveled(){
  return (getEncoderTics()/kPulsesPerRevolution) * kDistancePerRev; 
}

void Motor::setEncoderTics(int tics){
  tics_counter_=tics;
}

void Motor::PIDStraightTunnigs(double kp, double ki, double kd){
  pid_straight_.setTunnings(kp,ki,kd);
}
  
void Motor::PIDRotateTunnigs(double kp, double ki, double kd){
  pid_rotate_.setTunnings(kp,ki,kd);
}