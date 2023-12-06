#ifndef Motor_h
#define Motor_h

#include "Arduino.h"
#include "PID.h"


enum class MotorState
{
  Backward = -1,
  Stop = 0,
  Forward = 1
};


// checar el formato
enum class MotorID{
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT,
    NONE
};



class Motor{
    private:
        // MOTORES
        // __cpp_raw_strings motorID;
        uint8_t pwm = 0;
        uint8_t pwmPin;
        uint8_t digitalOne;
        uint8_t digitalTwo;
        MotorID motorId;
        // uint8_t speed = 0;
        double rpm;

        // ENCODERS
        uint8_t encoderA = 0;
        // uint8_t pidTics = 0;
        // static volatile int encoderTics;
        static volatile int encoderTicsFL;
        static volatile int encoderTicsFR;
        static volatile int encoderTicsBL;
        static volatile int encoderTicsBR;

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
        static constexpr double DistancePerRev = M_PI * kWheelsDiameter;



    public:
        Motor();

        Motor(uint8_t digitalOne, uint8_t digitalTwo, uint8_t encoderA, MotorID motorid);

        uint8_t getEncoderA();
        
        double getCurrentSpeed();

        double getTargetSpeed();

        double getTargetRps(double velocity);

        int getPidTics();

        void deltaPidTics(int deltaTics);

        int getEncoderTics();
        void deltaEncoderTics(int deltaTics);

        double RpmToRps(double velocity);

        double MsToRps(double ms);

        void motoresSetup(uint8_t pwmPin, uint8_t digitalOne, uint8_t digitalTwo, uint8_t encoderA, MotorID motorid);
        // static void updateTics();
        static void updateTicsFL();
        static void updateTicsFR();
        static void updateTicsBL();
        static void updateTicsBR();
        void initEncoder();
        
        MotorState getCurrentState();
        
        /*
        int getEncoderTicsFR();
        int getEncoderTicsBL();
        int getEncoderTicsBR();
        */
        // static void a(motor* motora);
        /*
        void setDirection(uint8_t direction);
        void setSpeed(uint8_t speed);
        void stop();
        */
        void updateRPM();

        void motorForward();

        void motorBackward();

        void motorStop();

        void setPWM(uint8_t pwm);

        double getPWM();

        void motorSpeedPID(double targetSpeed_, bool debug=false);

        void motorSpeedPWM(double targetSpeed_);

        void motorRotateDerPID(double targetAngle_, bool currentAngle_);

        void motorRotateIzqPID(double targetAngle_, bool currentAngle_);

        void PIDStraightTunings(double kp, double ki, double kd);

        void PIDRotateTunings(double kp, double ki, double kd);

        // -------------------------------------------------------------------
        void setPID(double targetSpeed, double kp, double ki, double kd);

        double getRPM();
};
#endif