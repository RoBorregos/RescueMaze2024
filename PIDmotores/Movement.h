#ifndef Movement_h
#define Movement_h

#include "Arduino.h"
#include "Motor.h"

class Movement{
    private:
        Motor motorFL;
        Motor motorFR;
        Motor motorBL;
        Motor motorBR;
        float RMPFL, RMPFR, RMPBL, RMPBR;
        int FLtics, FRtics, BLtics, BRtics;
        int FLticsViejos, FRticsViejos, BLticsViejos, BRticsViejos;
    public:
        Movement();
        void setup();
        void moveForward(int pwm);
        Motor getMotorFL();
        Motor getMotorFR();
        Motor getMotorBL();
        Motor getMotorBR();
        void updateRPM();
        float getRMPFL();
        float getRMPFR();
        float getRMPBL();
        float getRMPBR();
        /*void moveBackward();
        void turnLeft();
        void turnRight();
        void stop();*/
};

#endif