#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include "PID.h"

enum class MotorState{
  kBackward = -1,
  kStop = 0,
  kForward = 1
};

enum class MotorID {
    kBackLeft = 0,
    kFrontLeft = 1,
    kBackRight = 2,
    kFrontRight = 3,
    kNone
};

constexpr long long kOneSecInMs = 1000;
constexpr double kPulsesPerRev = 496.0;
constexpr double kPidCountTimeSampleInSec = 1000/100;
// 0.069 is the diameter of the blue wheels in meters
// 0.084 is the diameter of the black wheels in meters
constexpr double kWheelsDiameter = 0.0864;
constexpr double kDistancePerRev = M_PI * kWheelsDiameter;

class Motor {
    private:
        // PID
        PID pid_;
        constexpr static double kP_ = 150;
        constexpr static double kI_ = 100;
        constexpr static double kD_ = 0.0;
        constexpr static double minOutput_ = 0;
        constexpr static double maxOutput_ = 255;
        constexpr static double maxErrorSum_ = 4000;
        constexpr static long sampleTime_ = 100;

        // MOTORS
        uint8_t pwmPin_;
        uint8_t digitalOne_;
        uint8_t digitalTwo_;
        MotorID motorId_;

        double rpm_;

        // ENCODERS
        uint8_t encoderA_ = 0;
        static volatile int encoders_[4];

        // PID
        unsigned long nextTime_;
        double pwmInicial_;

        MotorState currentState_;

        unsigned long previousTime_ = 0;

        // This will be use for the total distance traveled.
        long long totalTics_ = 0;

        long long previousTics_ = 0;

        // This will be use for the PID and the speed and the variable can also be reset to 0 every lapse of time.
        long long timeEpochTics_ = 0;

        double currentSpeed_ = 0;
        double targetSpeed_ = 0;
        double previousSpeed_ = 0;

      
    public:
        Motor();

        Motor(uint8_t digitalOne, uint8_t digitalTwo, uint8_t pwmPin, uint8_t encoderA, MotorID motorid);

        uint8_t getEncoderA();
    
        long long getEpochTics();

        void deltaTics(const int deltaTics);

        long long getTotalTics();

        void deltaTotalTics(const int deltaTics);
        
        void initEncoder();
        
        MotorState getCurrentState();

        void motorForward(uint8_t pwm);

        void motorBackward(uint8_t pwm);

        void motorStop();

        void initMotor();

        void setEncoderTics(int tics);

        void setupMotor(const uint8_t pwmPin, const uint8_t digitalOne, const uint8_t digitalTwo, const uint8_t encoderA, const MotorID motorId);

        void setPwmAndDirection(const uint8_t pwm, const MotorState direction);

        void setSpeedAndDirection(const double speed, const MotorState direction);

        uint8_t speedToPwm(const double speed);

        double getCurrentSpeed(); // It returns the speed in meters per second (m/s)

        double getSpeed();

        void constantSpeed(const double speed, const MotorState direction);
};
#endif