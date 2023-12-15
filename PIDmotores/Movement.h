#ifndef Movement_h
#define Movement_h

#include "Motor.h"
#include "BNO.h"

enum class compass{
    knorth,
    keast,
    ksouth,
    kwest
};

enum class MovementState{
    kStop,
    kForward,
    kBackward,
    kTurnLeft,
    kTurnRight
};

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
        void forwardMotors(const uint8_t pwms[4]);
        void backwardMotors(const uint8_t pwms[4]);

        void turnRightMotors(const uint8_t pwms[4]);
        void turnLeftMotors(const uint8_t pwms[4]);

        void forwardMotor(const uint8_t pwm, MotorID motorId);
        void setPwmsAndDirections(const uint8_t pwms[4], const MotorState directions[4]);
        

        //void moveMotors(MotorState state);

        void updateTics(MotorID motorId);

        int getBackLeftEncoderTics();
        int getBackRightEncoderTics();
        int getFrontLeftEncoderTics();
        int getFrontRightEncoderTics();    

        int getOrientation(const compass currentOrientation);
        //void computeTargetOrientation(compass targetOrientation, compass currentOrientation);
        void moveMotors(MovementState state, const double targetOrientation);

        

};

#endif