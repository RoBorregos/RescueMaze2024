#ifndef Movement_h
#define Movement_h

#include "Arduino.h"
#include "Motor.h"
#include "BNO.h"

class Movement{
    private:
        Motor motorFL;
        Motor motorFR;
        Motor motorBL;
        Motor motorBR;
        /*float RMPFL, RMPFR, RMPBL, RMPBR;
        int FLtics, FRtics, BLtics, BRtics;
        int FLticsViejos, FRticsViejos, BLticsViejos, BRticsViejos;*/
        unsigned long next_time;
        // float kp, ki, kd;
        /*float pwmInicialFL,errorPrevFL;
        float pwmInicialFR,errorPrevFR;
        float pwmInicialBL,errorPrevBL;
        float pwmInicialBR,errorPrevBR;*/
        float errorPrevOrientation;
        float errorAcumuladoOrientation;
    public:
        Movement();
        void setup();
        void moveForward(int pwmA, int pwmB, int pwmC, int pwmD);
        void updateRPM();
        void setSpeed(float targetSpeed,float orientation,BNO bno);
        //getters
        Motor getMotorFL();
        Motor getMotorFR();
        Motor getMotorBL();
        Motor getMotorBR();
        float getRPMFL();
        float getRPMFR();
        float getRPMBL();
        float getRPMBR();

        float getPWMInicialFL();
        float getPWMInicialFR();
        float getPWMInicialBL();
        float getPWMInicialBR();
        /*void moveBackward();
        void turnLeft();
        void turnRight();
        void stop();*/
};

#endif