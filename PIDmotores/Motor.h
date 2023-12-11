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
enum class MotorID {
    BACK_LEFT = 0,
    FRONT_LEFT = 1,
    BACK_RIGHT = 2,
    FRONT_RIGHT = 3,
    NONE
};

class Motor {
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
        static constexpr double DistancePerRev = M_PI * kWheelsDiameter;



    public:
        Motor();

        Motor(uint8_t digitalOne, uint8_t digitalTwo, uint8_t pwmPin, uint8_t encoderA, MotorID motorid);

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

        void motorSetup();
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

        void motorForward(uint8_t pwm);

        void motorBackward(uint8_t pwm);

        void motorStop();

        void setPWM(uint8_t pwm);

        int getPWM();

        void motorSpeedPID(double targetSpeed_, bool debug=false);

        void motorSpeedPWM(double targetSpeed_);

        void motorRotateDerPID(double targetAngle_, double currentAngle_);

        void motorRotateIzqPID(double targetAngle_, double currentAngle_);

        void PIDStraightTunings(double kP, double kI, double kD);

        void PIDRotateTunings(double kP, double kI, double kD);

        // -------------------------------------------------------------------
        // Warning: This function will be deleted 
        void setPID(double targetSpeed, double kP, double kI, double kD);
        // -------------------------------------------------------------------------------
        double getRPM();

        void initMotor();

        double getDistanceTraveled();

        void setEncoderTics(int tics);

        void motoresSetup(uint8_t pwmPin, uint8_t digitalOne, uint8_t digitalTwo, uint8_t encoderA, MotorID motorId);
};
#endif