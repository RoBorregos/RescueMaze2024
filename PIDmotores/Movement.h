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
        PID pid_;
        Motor motorFL_;
        Motor motorFR_;
        Motor motorBL_;
        Motor motorBR_;

        unsigned long nextTime_;
  
        float errorPrevOrientation_;
        float errorAcumuladoOrientation_;

        long long int ticsCounter_ = 0;
        int pidTics_ = 0;

        double currentSpeed_ = 0;
        double targetSpeed_ = 0;

        Motor motor[4];

    public:
        Movement();

        void setup();
        
        void setupInternal(MotorID motorId);

        void stopMotors();

        void setPwmsAndDirections(const uint8_t pwms[4], const MotorState directions[4]);
        
        void updateTics(MotorID motorId);

        double getBackLeftSpeed();
        double getBackRightSpeed();
        double getFrontLeftSpeed();
        double getFrontRightSpeed();    

        int getOrientation(const compass currentOrientation);
        void moveMotors(const MovementState state, const double targetOrientation);

        void setMotorsDirections(const MovementState state, MotorState directions[4]);

        void setSpeed(const double speed);
};

#endif