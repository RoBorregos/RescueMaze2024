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

        static constexpr double kMaxOrientationError = 0.8;
        static constexpr uint8_t kNumberOfWheels = 4;

        PID pidForward;
        PID pidBackward;
        PID pidTurn;

        constexpr static double kPForward = 0.015; 
        constexpr static double kIForward = 0.00;
        constexpr static double kDForward = 0.0;

        constexpr static double kPBackward = 0.02;
        constexpr static double kIBackward = 0.0;
        constexpr static double kDBackward = 0.0;

        constexpr static double kPTurn = 0.010;
        constexpr static double kITurn = 0.0;
        constexpr static double kDTurn = 0.008;

        constexpr static double kMaxErrorSum{4000};
        constexpr static double kMinOutput{0};
        constexpr static double kTurnMinOutput{0.05};
        constexpr static double kMaxOutput{0.5};
        constexpr static long kSampleTime{100};

    public:
        Movement();

        void setup();
        
        void setupInternal(MotorID motorId);

        void stopMotors();

        void setPwmsAndDirections(const uint8_t pwms[4], const MotorState directions[4]);

        void setSpeedsAndDirections(const double speeds[4], const MotorState directions[4]);
        
        void updateTics(MotorID motorId);

        double getBackLeftSpeed();
        double getBackRightSpeed();
        double getFrontLeftSpeed();
        double getFrontRightSpeed();    

        uint8_t getOrientation(const compass currentOrientation);
        bool moveMotors(const MovementState state, const double targetOrientation);

        void setMotorsDirections(const MovementState state, MotorState directions[4]);

        void setSpeed(const double speed);
};

#endif