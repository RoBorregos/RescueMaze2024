#ifndef Motores_h
#define Motores_h

#include <Arduino.h>
#include "PID.h"

struct MotorID{
    backLeft=0;
    backRight=1;
    frontLeft=2;
    frontRight=3;
};

enum class MotorState{
    forward = 1,
    backward = -1,
    stop = 0
};

class Motor{

//VERIFICAR DATOS .....................................................................................................................
    private:
    //Motor info
    uint8_t digital_one_;
    uint8_t digital_two_;
    int pwm_pin_;
    uint8_t encoderA_;
    uint8_t encoderB_;
    MotorID motorID_;
    MotorState current_state_;

    //VelocityData
    uint8_t pwm_ = 0;
    long long int tics_counter_ = 0;
    int pid_tics_ = 0;
    double current_speed_ = 0;
    double target_speed_ = 0;

  
    //Motor characteristics.
    static constexpr double kPulsesPerRevolution = 496.0;
    static constexpr double kWheelDiameter = 0.069;
    static constexpr double kRPM = 150;
    static constexpr double kRPS = kRPM / 60;
    static constexpr double kMaxVelocity = kRPS * M_PI * kWheelDiameter;
    static constexpr double kMaxPWM = 255;
    static constexpr double kPwmDeadZone = 0;
    static constexpr double kMinPwmForMovement = 0;
    static constexpr double kDistancePerRev = M_PI * kWheelDiameter;

    // PID.
    static constexpr uint8_t kPidMinOutputLimit = 30;
    static constexpr uint8_t kPidMaxOutputLimit = 255;
    static constexpr uint16_t kPidMaxErrorSum = 2000;
    static constexpr uint8_t kPidMotorTimeSample = 100;
    static constexpr double kOneSecondInMillis = 1000.0;
    static constexpr double kSecondsInMinute = 60;
    static constexpr double kPidCountTimeSamplesInOneSecond = kOneSecondInMillis/kPidMotorTimeSample;
    static constexpr double kPidCountTimeSamplesInOneMinute = kSecondsInMinute*kPidCountTimeSamplesInOneSecond;

    PID pid_straight_;
    static constexpr double kPStraight = 13; //60
    static constexpr double kIStraight = 6; //55
    static constexpr double kDStraight = 2; //40
    PID pid_rotate_;
    static constexpr double kPRotate = 1.25; //0.5
    static constexpr double kIRotate = 0;
    static constexpr double kDRotate = 0;

  public:
    
    
    //Constructor
    Motor(uint8_t digitalOne, uint8_t digitalTwo, int pwmPin, uint8_t encoderA, uint8_t entcoderB, MotorID motorID);
    Motor();

    //Getters
    MotorState getCurrentState();
    double getCurrentSpeed();
    double getTargetSpeed();
    int getPidTics();

    //Encoders
    uint8_t getEncoderB();
    uint8_t getEncoderA();
    int getEncoderTics();
    void setEncoderTics(int tics);
    double getDistanceTraveled();

    //Methods
    void motorSetup();
    void deltaEncoderTics(int delta);
    void deltaPidTics(int delta);

    // Attach interrupt of encoders.
    void initEncoders();
    
    //Control
    void motorForward();
    void motorBackward();
    void motorStop();

    //Velocity Methods
    void setPWM(double PWM);
    double RPM2RPS(double velocity);
    double getTargetRps(double velocity);
    double Ms2Rps(double MS);
    double Pwm2Rpm(double pwm);
    void motorSpeedPID(double target_speed);
    void motorRotateDerPID(double target_angle, double current_angle);
    void motorRotateIzqPID(double target_angle, double current_angle);
    void motorSpeedPWM(double target_speed);
    double getPWM();

    //PID METHODS
    void PIDStraightTunnigs(double kp, double ki, double kd);
    void PIDRotateTunnigs(double kp, double ki, double kd);
}




#endif
