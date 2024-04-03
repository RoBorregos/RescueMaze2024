#ifndef Movement_h
#define Movement_h

#include "Motor.h"
#include "BNO.h"
#include "VLX.h"
#include "LimitSwitch.h"
#include "TCS.h"

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
        static constexpr double kBaseSpeedForward_ = 0.15; // m/s
        static constexpr double kBaseSpeedTurn_ = 0.07; // m/s
        static constexpr double kBaseSpeedForwardReset_ = 0.05; // m/s

        static constexpr uint8_t kMaxMovements_ = 4;

        static constexpr double kMinFrontWallDistance = 0.06;

        // TODO: Write the member variables like this kNumberOfVlx_ and kMToCm_

        static constexpr uint8_t kNumberOfLimitSwitches = 2;

        LimitSwitch limitSwitch_[kNumberOfLimitSwitches];

        static constexpr uint8_t kNumberOfVlx = 5;
        static constexpr double kMToCm = 100.0;
        static constexpr uint8_t kVlxOffset = 2; //cm

        static constexpr double kIdealDistanceCenter = 0.05;

        static constexpr uint8_t kTileLength = 30; //cm

        static constexpr double kTileLengthInMeters = 0.3; //m

        static constexpr double kHalfTile = 15.0; //cm

        static constexpr double kHalfTileInMeters = 0.15; //m

        static constexpr double kLengthOfRobot = 19.9; //cm

        bool resetDistanceTraveled_ = false;

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

        static constexpr double kMToCenter_ = 0.05; // 5 cm

        static constexpr double kMinWallDistance = 0.15; // 15 cm

        static constexpr double kMaxDistanceError = 0.03; // 3 cm

        static constexpr double kMaxOrientationError = 0.9;

        static constexpr double kMinRampOrientation = 10.0;

        static constexpr double kMaxDistanceError = 0.01;

        static constexpr long long kOneSecInMs = 1000;

        double targetOrientation_ = 0;

        static constexpr double kUnreachableDistance = 0.7;

        static constexpr double kWeightEncoders = 0.4;

        static constexpr double kWeightVlx = 0.6;

        static constexpr double kWeightBNO = 0.5;

        static constexpr double kWeightVLX = 0.5;

        int counterMovements_ = 0;

        bool encodersReset_ = false;

        bool inResetRoutine_ = false;

        PID pidDummy_;
        PID pidForward_;
        PID pidBackward_;
        PID pidTurn_;
        PID pidWallAlignment_;

        double vlxDistanceTraveled_;

        double distanceToCenter_;

        constexpr static double kPForward = 0.015; 
        constexpr static double kIForward = 0.00;
        constexpr static double kDForward = 0.002;

        constexpr static double kPBackward = 0.02;
        constexpr static double kIBackward = 0.0;
        constexpr static double kDBackward = 0.0;

        constexpr static double kPTurn = 0.00005;
        constexpr static double kITurn = 0.0;
        constexpr static double kDTurn = 0.00019;

        constexpr static double kPDistance = 0.01;
        constexpr static double kIDistance = 0.0;
        constexpr static double kDDistance = 0.0;

        constexpr static double kMaxErrorSum{4000};
        constexpr static double kMinOutput{0};
        constexpr static double kTurnMinOutput{0.050};
        constexpr static double kMaxOutput{0.5};
        constexpr static long kSampleTime{100};
        constexpr static long kSampleTimeTraveled{50};

        // TCS
        TCS tcs_;
        static constexpr int kPrecision = 100;
        static constexpr uint8_t kColorAmount = 3;
        static constexpr uint8_t kColorThresholdsAmount = 6;
        const char kColorList[kColorAmount + 1] = {"RNB"};
        static constexpr char kBlueColor = 'B';
        static constexpr char kBlackColor = 'N';
        static constexpr char kRedColor = 'R';

        // ==============================================================================================
        // In the competition the colors may be different so we need to change the values by testing
        // ==============================================================================================
        const int16_t kColors[kColorAmount][kColorAmount] = {
            // RED
            {257, 75, 71},
            // BLACK
            {80, 44, 34},
            // BLUE
            {97,99,141}
        };
        
        const int16_t kColorThresholds[kColorAmount][kColorThresholdsAmount] {
            {220, 270, 60, 80, 50, 75},
            {20, 120, 33, 90, 25, 79},
            {85, 150, 80, 140, 120, 175}
        };

        bool blackTile_ = false;
        bool blueTile_ = false;
        bool checkpointTile_ = false;

        bool finishedMovement_ = false;

        static constexpr int kFiveSeconds_ = 5000;

    public:
        Movement();

        void setup();
        
        void setupInternal(const MotorID motorId);

        void setupLimitSwitch(const LimitSwitchID limitSwitchId);

        void setupTCS();

        void stopMotors();

        void setPwmsAndDirections(const uint8_t pwms[4], const MotorState directions[4]);

        void setSpeedsAndDirections(const double speeds[4], const MotorState directions[4]);
        
        void updateTics(MotorID motorId);

        double getBackLeftSpeed();
        double getBackRightSpeed();
        double getFrontLeftSpeed();
        double getFrontRightSpeed();

        uint8_t getOrientation(const compass currentOrientation);
        
        void moveMotors(const MovementState state, const double targetOrientation, double targetDistance, bool useWallDistance = true);

        void setMotorsDirections(const MovementState state, MotorState directions[4]);

        void setSpeed(const double speed);

        bool hasTraveledDistanceWithSpeedForBackward(const double distance);

        bool hasTraveledDistanceWithSpeed(const double distance);

        bool hasTraveledWallDistance(const double targetDistance, const double currentDistance);

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

        void printTCS();

        char getTCSInfo();

        void rgbTCSClear();

        char checkColors();

        bool isRamp();

        void rampMovement();

        double weightMovement(const double currentDistanceBack, const double currentDistanceFront, const double initialVlxDistanceBack, const double initialVlxDistanceFront);

        void updateDistanceToCenterInTile();

        bool hasWallBehind();

        void maybeResetWithBackWall(const double targetOrientation, const double currentOrientation);

        double getPhaseCorrection(const double currentOrientation, const double targetOrientation);

        double getRealTargetDistance(const double targetDistance);

        bool wasBlackTile();

        bool isBlueTile();

        bool isCheckpointTile();

        void weightPID(const double targetOrientation, const double currentOrientation, const double targetDistance, const double currentDistance, const double& speedLeft, const double& speedRight);
};

#endif