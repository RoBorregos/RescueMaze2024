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
    public:
        Motor();
        void motoresSetup(uint8_t pwmPin, uint8_t digitalOne, uint8_t digitalTwo, uint8_t encoderA, MotorID motorid);
        // static void updateTics();
        static void updateTicsFL();
        static void updateTicsFR();
        static void updateTicsBL();
        static void updateTicsBR();
        void initEncoder();
        int getEncoderTics();
        /*
        int getEncoderTicsFR();
        int getEncoderTicsBL();
        int getEncoderTicsBR();
        */
        // static void a(motor* motora);
        void setPWM(uint8_t pwm);
        /*
        void setDirection(uint8_t direction);
        void setSpeed(uint8_t speed);
        void stop();
        */
        void updateRPM();
        void setPID(double targetSpeed, double kp, double ki, double kd);
        double getPWM();
        double getRPM();
};
#endif