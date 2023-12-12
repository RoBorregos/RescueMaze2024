#ifndef Movement_h
#define Movement_h

#include "Motor.h"
#include "BNO.h"

class Movement {
    private:
        Motor motorFL;
        Motor motorFR;
        Motor motorBL;
        Motor motorBR;

        unsigned long next_time;
  
        float errorPrevOrientation;
        float errorAcumuladoOrientation;

        long long int ticsCounter=0;
        int pidTics = 0;

        double currentSpeed = 0;
        double targetSpeed = 0;

        Motor motor[4];

    public:
        Movement();

        void setup();
        
        void setupInternal(MotorID motorId);

        void stopMotors();
        void forwardMotors(uint8_t pwms[4]);
        void backwardMotors(uint8_t pwms[4]);

        void moveMotors(MotorState state);

        void updateTics(MotorID motorId);

        int getBackLeftEncoderTics();
        int getBackRightEncoderTics();
        int getFrontLeftEncoderTics();
        int getFrontRightEncoderTics();        
};

#endif