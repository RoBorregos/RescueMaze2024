#ifndef Movement_h
#define Movement_h

#include "Motor.h"
#include "BNO.h"

enum class compass{
    kNorth,
    kEast,
    kSouth,
    kWest
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

        void setPwmsAndDirections(const uint8_t pwms[4], const MotorState directions[4]);
        

        //void moveMotors(MotorState state);

        void updateTics(MotorID motorId);

        int getBackLeftEncoderTics();
        int getBackRightEncoderTics();
        int getFrontLeftEncoderTics();
        int getFrontRightEncoderTics();    

        int getOrientation(const compass currentOrientation);
        //void computeTargetOrientation(compass targetOrientation, compass currentOrientation);
        void moveMotors(const MovementState state, const double targetOrientation);

        void setMotorsDirections(const MovementState state, MotorState directions[4]);
        


};

#endif