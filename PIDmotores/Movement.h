#ifndef Movement_h
#define Movement_h

#include "Motor.h"
#include "BNO.h"
#include "VLX.h"
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
        const double kMToCm = 100.0;
        const uint8_t kVlxOffset = 2; //cm
        const uint8_t kTileLength = 30; //cm

        const double kHalfTile = 15.0; //cm

        const double kLargeOfRobot = 19.9; //cm

        double cmToCenterFront = 0;

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

        double wallDistances[kNumberOfVlx];

        const double kMinWallDistance = 0.0775; // 7.75 cm

        static constexpr double kMaxDistanceError = 0.03;

        static constexpr double kMaxOrientationError = 0.9;

        static constexpr double kMinRampOrientation = 17.0;

        static constexpr long long kOneSecInMs = 1000;

        double targetOrientation_ = 0;

        const double kUnreachableDistance = 1.0;

        static constexpr double kWeightEncoders = 0.1;

        static constexpr double kWeightVlx = 0.9;

        int counterMovements_ = 0;

        bool encodersReset = false;

        bool resetRoutine = false;

        PID pidDummy_;
        PID pidForward_;
        PID pidBackward_;
        PID pidTurn_;

        double vlxDistanceTraveled_;

        double distanceToCenter_;

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

        bool hasTraveledDistanceWithSpeedForBackward(const double distance);

        bool hasTraveledDistanceWithSpeed(const double distance);

        bool hasTraveledWallDistance(const double targetDistance, const double currentDistance, bool &moveForward, double initialVlxDistance);

        void moveMotorsInADirection(double targetOrientation, bool moveForward);

        void setupVlx(const VlxID vlxId);

        void getAllWallsDistances(double wallDistances[kNumberOfVlx]);

        uint8_t checkWallsDistances();

        double getDistanceToCenter();

        double getWallDistance(const VlxID vlxId);

        void goForward();

        void goBackward();

        void turnLeft();

        void turnRight();

        void turnMotors(const double targetOrientation, const double targetDistance, double &currentOrientation);

        MovementState getCurrentState();

        void saveLastState(const MovementState state, double &targetOrientation);

        void retrieveLastState();

        void correctionAfterCrash(const bool crashSide, double currentOrientation, bool useWallDistance);

        double getOrientation(const double orientation);

        bool isRamp();

        void rampMovement();

        double weightMovemnt(double currentDistanceBack, double currentDistanceFront, double initialVlxDistanceBack, double initialVlxDistanceFront);

        bool centerInTile();

        bool hasWallBehind();

        void resetOrientation();
    };

#endif