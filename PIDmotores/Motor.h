#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include "PID.h"

enum class MotorState{
  kBackward = -1,
  kStop = 0,
  kForward = 1
};

// checar el formato
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
constexpr double kWheelsDiameter = 0.069;
constexpr double kDistancePerRev = M_PI * kWheelsDiameter;

class Motor {
    private:
        PID pid;
        // MOTORS
        uint8_t pwm = 0;
        uint8_t pwmPin;
        uint8_t digitalOne;
        uint8_t digitalTwo;
        MotorID motorId;

        double rpm;

        // ENCODERS
        uint8_t encoderA = 0;
        static volatile int encoders[4];

        // PID
        unsigned long next_time;
        double pwmInicial;
        double errorPrev;
        double errorAcumulado;

        MotorState currentState_;

        unsigned long previousTime = 0;

        // This will be use for the total distance traveled.
        long long totalTics = 0;


        // This will be use for the PID and the speed and the variable can also be reset to 0 every lapse of time.
        long long timeEpochTics = 0;

        double currentSpeed_ = 0;
        double targetSpeed = 0;
        double previousSpeed = 0;

        // static constexpr long long kOneSecInMs = 1000;

        // TODO: Motor characteristics
        // ...........................
       /*  static constexpr double kPulsesPerRev = 496.0;
        static constexpr double kPidCountTimeSampleInSec = 1000/100;
        static constexpr double kWheelsDiameter = 0.069;
        static constexpr double kDistancePerRev = M_PI * kWheelsDiameter; */

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
        
        double getRPM();

        void initMotor();

        void setEncoderTics(int tics);

        void motorSetup(const uint8_t pwmPin, const uint8_t digitalOne, const uint8_t digitalTwo, const uint8_t encoderA, const MotorID motorId);

        void setPwmAndDirection(const uint8_t pwm, const MotorState direction);

        double getCurrentSpeed(); // devuelve velocidad en metros por segundo

        double getSpeed();

        double ticsToMs();

        //static double ticsToSpeed(const long long tics, const unsigned long time);

        void constantSpeed(const double speed);
};
#endif