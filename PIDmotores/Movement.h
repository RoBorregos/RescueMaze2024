#ifndef Movement_h
#define Movement_h

#include "Motor.h"
#include "BNO.h"
#include "VLX.h"
#include "TileDirection.h"
#include "LimitSwitch.h"
#include "TCS.h"
#include <Deneyap_Servo.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

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

enum class servoPosition {
    kLeft,
    kRight,
    kCenter
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
        static constexpr double kBaseSpeedForward_ = 0.09; // m/s 0.09
        static constexpr double kBaseSpeedTurn_ = 0.07; // m/s
        static constexpr double kBaseSpeedForwardReset_ = 0.05; // m/s


        static constexpr uint8_t kMaxMovements_ = 4;

        // TODO: Improve the name of this variable
        static constexpr double kMinWallDistance = 0.06; // 6 cm

        // TODO: Write the member variables like this kNumberOfVlx_ and kMToCm_

        static constexpr uint8_t kNumberOfLimitSwitches = 2;

        LimitSwitch limitSwitch_[kNumberOfLimitSwitches];

        static constexpr uint8_t kNumberOfVlx = 5;

        // static constexpr uint8_t kOffArray = -1;

        // static constexpr int kTargetOrientations[] = {0, 90, 180, 270};
        const int kTargetOrientations[4] = {0, 90, 180, 270};

        static constexpr uint8_t kNumberOfTargetOrientations = 4;

        static constexpr double kMToCm = 100.0;
        static constexpr uint8_t kVlxOffset = 2; //cm

        static constexpr double kIdealDistanceCenter = 0.05; // 5 cm

        static constexpr uint8_t kTileLength = 30; //cm

        const bool kOffArray = false;

        static constexpr double kTileLengthInMeters = 0.3; //m

        static constexpr double kHalfTile = 15.0; //cm

        static constexpr double kHalfTileInMeters = 0.15; //m

        static constexpr double kLengthOfRobot = 19.9; //cm

        bool resetDistanceTraveled_ = false;

        bool useWallDistance_ = false;

        unsigned long kTimeAfterRamp = 1400;

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

        // TODO: Improve the name of this variable
        static constexpr double kWallDistance = 0.18; // 18 cm

        static constexpr uint8_t kMaxDistanceErrorInCm = 1; // 1 cm

        static constexpr double kMaxDistanceErrorInMeters = 0.01; // 1 cm

        static constexpr double kMaxOrientationError = 0.9;

        static constexpr double kMinRampOrientation = 10.0;

        static constexpr long long kOneSecInMs = 1000;

        static constexpr double kOneTileDistance = 0.3; // 0.228 m

        static constexpr double kMoveOneTileDistance = 0.30; // 0.228 m

        static constexpr long long kTileDirections = 4;

        double targetOrientation_ = 0;

        static constexpr double kUnreachableDistance = 0.85;

        static constexpr double kUnreachableDistanceInCm = 85;

        static constexpr double kWeightEncoders = 0.0;

        static constexpr double kWeightVlx = 1.0;

        int counterMovements_ = 0;

        bool encodersReset_ = false;

        bool inResetRoutine_ = false;


        PID pidDummy_;
        PID pidForward_;
        PID pidBackward_;
        PID pidTurn_;

        double vlxDistanceTraveled_;

        double distanceToCenter_;

        constexpr static double kPForward = 0.07; // 0.09
        constexpr static double kIForward = 0.00;
        constexpr static double kDForward = 0.00;

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
        static constexpr int kTwoSeconds_ = 2000;

        // Dispenser stuff.
        // const char kCheckpointSerialCode = -1;
        const char kHarmedSerialCode = 'h';
        const char kStableSerialCode = 's';
        const char kUnharmedSerialCode = 'u';
        const char kNoVictimSerialCode = 'm';
        const int kHalfSecond = 500;
        char victim = 'm';
        bool hasReceivedSerial = false;
        Servo myservo;
        static constexpr uint8_t initialAngle = 80;
        static constexpr uint8_t rightAngle = initialAngle + 44;
        static constexpr uint8_t leftAngle = initialAngle - 36;
        bool victimFound = false;
        u_int8_t rightStock = 6;
        u_int8_t leftStock = 6;

        Adafruit_SSD1306 display;

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
        
        void moveMotors(const MovementState state, const double targetOrientation, double targetDistance, bool useWallDistance = true, bool center = false);

        void setMotorsDirections(const MovementState state, MotorState directions[4]);

        void setSpeed(const double speed);

        bool hasTraveledDistanceWithSpeedForBackward(const double distance);

        bool hasTraveledDistanceWithSpeed(const double distance);

        bool hasTraveledWallDistance(const double targetDistance, const double currentDistance);

        void moveMotorsInADirection(double targetOrientation, bool moveForward);

        void setupVlx(const VlxID vlxId);

        void getAllWallsDistances(double wallDistances[kNumberOfVlx]);

        uint8_t checkWallsDistances();

        bool checkWallsDistances(const TileDirection targetTileDirection, const double currentOrientation);

        double getDistanceToCenter();

        double getWallDistance(const VlxID vlxId);

        void goForward(const double targetOrientation, const bool& hasVictim);

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

        void printTCS();

        char getTCSInfo();

        void rgbTCSClear();

        char checkColors(const double targetOrientation);

        bool isRamp();

        void rampMovement(const double targetOrientation);

        int directionRamp();

        double weightMovement(const double currentDistanceBack, const double currentDistanceFront, const double initialVlxDistanceBack, const double initialVlxDistanceFront);

        double weightMovementVLX(const double currentDistanceBack, const double currentDistanceFront, const double initialVlxDistanceBack, const double initialVlxDistanceFront, bool& moveForward, double targetDistance);

        void updateDistanceToCenterInTile();

        bool hasWallBehind();

        bool hasWallInFront();

        void maybeResetWithBackWall(const double targetOrientation, const double currentOrientation);

        double getPhaseCorrection(const double currentOrientation, const double targetOrientation);

        double getRealTargetDistance(const double targetDistance);

        bool wasBlackTile();

        bool isBlueTile();

        bool isCheckpointTile();

        double getDistanceToCenter(double targetDistance);

        void sendSerialRequest();

        void checkSerial(double currentOrientation);

        char getVictim();

        void moveServo(servoPosition position);

        bool getVictimFound();

        void screenPrint(const String output);

        void checkForCrashAndCorrect(bool crashLeft, bool crashRight, double currentOrientation, bool useWallDistance);
};

#endif