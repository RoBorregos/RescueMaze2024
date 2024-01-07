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

class Motor {
    private:
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

        MotorState currentState;

        long long int ticsCounter=0;
        int pidTics = 0;

        double currentSpeed = 0;
        double targetSpeed = 0;

        // TODO: Motor characteristics
        // ...........................
        static constexpr double kPulsesPerRev = 496.0;
        static constexpr double kPidCountTimeSampleInSec = 1000/100;
        static constexpr double kWheelsDiameter = 0.069;
        static constexpr double kDistancePerRev = M_PI * kWheelsDiameter;

    public:
        Motor();

        Motor(uint8_t digitalOne, uint8_t digitalTwo, uint8_t pwmPin, uint8_t encoderA, MotorID motorid);

        uint8_t getEncoderA();
    
        int getPidTics();

        void deltaPidTics(int deltaTics);

        long long getEncoderTics();

        void deltaEncoderTics(int deltaTics);
        
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
        
        void constSpeed(const double speed); // Darle velocidad en metros por segundo

        double getSpeed();

        void ticsToMs ();
};
#endif