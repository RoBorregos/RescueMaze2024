#ifndef Movement_h
#define Movement_h

#include "Motor.h"
#include "BNO.h"
#include "VLX.h"
#include "TileDirection.h"
#include "LimitSwitch.h"

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
    kTurnRight,
    kRamp
};

class Movement {
    private:
        PID pid_;
        BNO bno_;

        MovementState currentState_;
        MovementState lastState_;

        double crashDeltaOrientation_ = 20.0;
        double crashDeltaDistance_ = 0.02;

        unsigned long prevTimeTraveled_;
        double allDistanceTraveled_ = 0; 
  
        float errorPrevOrientation_;
        float errorAcumuladoOrientation_;

        long long ticsCounter_ = 0;
        long long pidTics_ = 0;

        double currentSpeed_ = 0;
        double targetSpeed_ = 0;
        static constexpr double kBaseSpeedForward_ = 0.14;
        static constexpr double kBaseSpeedTurn_ = 0.07;

        // TODO: Write the member variables like this kNumberOfVlx_ and kMToCm_

        static constexpr uint8_t kNumberOfLimitSwitches = 2;

        LimitSwitch limitSwitch_[kNumberOfLimitSwitches];

        static constexpr uint8_t kNumberOfVlx = 5;

        // static constexpr uint8_t kOffArray = -1;

        // static constexpr int kTargetOrientations[] = {0, 90, 180, 270};
        const int kTargetOrientations[4] = {0, 90, 180, 270};

        static constexpr uint8_t kNumberOfTargetOrientations = 4;

        const double kMToCm = 100.0;
        const uint8_t kVlxOffset = 2; //cm
        const uint8_t kTileLength = 30; //cm

        const bool kOffArray = false;

        bool useWallDistance_ = false;

        const unsigned long kTimeAfterRamp = 750;

        double currentDistance_ = 0;
        double targetDistance_ = 0;
        double distancePrev_ = 0;

        double crashDistance_ = 0;

        double timePrev_ = 0;

        VlxID vlxDirections[kNumberOfVlx] = {VlxID::kFrontRight, VlxID::kBack, VlxID::kLeft, VlxID::kRight, VlxID::kFrontLeft};

        double sampleTime_ = 100;

        static constexpr uint8_t kNumberOfWheels = 4;

        Motor motor[kNumberOfWheels];

        VLX vlx[kNumberOfVlx];

        double wallDistances_[kNumberOfVlx];

        const double kMinWallDistance = 0.15; // 7.75 cm 0.0775 m

        static constexpr double kMaxDistanceError = 0.02;

        static constexpr double kMaxOrientationError = 0.9;

        static constexpr double kMinRampOrientation = 17.0;

        static constexpr long long kOneSecInMs = 1000;

        static constexpr double kOneTileDistance = 0.3; //m

        static constexpr long long kTileDirections = 4;

        double targetOrientation_ = 0;

        PID pidDummy_;
        PID pidForward_;
        PID pidBackward_;
        PID pidTurn_;

        constexpr static double kPForward = 0.015; 
        constexpr static double kIForward = 0.00;
        constexpr static double kDForward = 0.0;

        constexpr static double kPBackward = 0.02;
        constexpr static double kIBackward = 0.0;
        constexpr static double kDBackward = 0.0;

        constexpr static double kPTurn = 0.00005;
        constexpr static double kITurn = 0.0;
        constexpr static double kDTurn = 0.00019;

        constexpr static double kMaxErrorSum{4000};
        constexpr static double kMinOutput{0};
        constexpr static double kTurnMinOutput{0.050};
        constexpr static double kMaxOutput{0.5};
        constexpr static long kSampleTime{100};
        constexpr static long kSampleTimeTraveled{50};

    public:
        Movement();

        void setup();
        
        void setupInternal(const MotorID motorId);

        void setupLimitSwitch(const LimitSwitchID limitSwitchId);

        void stopMotors();

        void setPwmsAndDirections(const uint8_t pwms[4], const MotorState directions[4]);

        void setSpeedsAndDirections(const double speeds[4], const MotorState directions[4]);
        
        void updateTics(MotorID motorId);

        double getBackLeftSpeed();
        double getBackRightSpeed();
        double getFrontLeftSpeed();
        double getFrontRightSpeed();    

        uint8_t getOrientation(const compass currentOrientation);
        
        void moveMotors(const MovementState state, const double targetOrientation, const double targetDistance, bool useWallDistance = true);

        void setMotorsDirections(const MovementState state, MotorState directions[4]);

        void setSpeed(const double speed);

        bool hasTraveledDistanceWithSpeed(const double distance);

        bool hasTraveledWallDistance(const double targetDistance, const double currentDistance, bool &moveForward);

        void moveMotorsInADirection(double targetOrientation, bool moveForward);

        void setupVlx(const VlxID vlxId);

        void getAllWallsDistances(double wallDistances[kNumberOfVlx]);

        uint8_t checkWallsDistances();

        double getDistanceToCenter();

        double getWallDistance(const VlxID vlxId);

        void goForward(const double targetOrientation);

        void goBackward(const double targetOrientation);

        void turnLeft(const double targetOrientation);

        void turnRight(const double targetOrientation);

        void turnMotors(const double targetOrientation, const double targetDistance, double &currentOrientation);

        MovementState getCurrentState();

        void saveLastState(const MovementState state, double &targetOrientation);

        void retrieveLastState();

        void correctionAfterCrash(const bool crashSide, double currentOrientation, bool useWallDistance);

        double getOrientation(const double orientation);

        int8_t getIndexFromArray(const int value, const int array[], const uint8_t arraySize);

        bool checkWallsDistances(const TileDirection targetDirection, const double currentOrientation);
        bool isRamp();

        void rampMovement();
};

#endif