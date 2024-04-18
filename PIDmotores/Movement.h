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
        double crashDeltaDistance_ = 0.10;

        unsigned long prevTimeTraveled_;
        double allDistanceTraveled_ = 0; 
  
        float errorPrevOrientation_;
        float errorAcumuladoOrientation_;

        long long ticsCounter_ = 0;
        long long pidTics_ = 0;

        int counterCollisionsLeft_ = 0;
        int counterCollisionsRight_ = 0;
        // CHECK IF WE NEED TO CHANGE THIS IN THE COMPETENCE
        int maxCollisions_ = 100;

        const unsigned long kTimeToDetectBumper_ = 2000;

        double timePrevBumper_ = 0;

        bool bumperDetected_ = false;

        static constexpr double kOrientationError = 10.0;

        double currentSpeed_ = 0;
        double targetSpeed_ = 0;
        static constexpr double kBaseSpeedForward_ = 0.25; // m/s 0.09
        static constexpr double kBaseSpeedTurn_ = 0.1; // m/s 0.07
        static constexpr double kBaseSpeedRoutine_ = 0.17; // m/s 0.17
        static constexpr double kBaseSpeedForwardReset_ = 0.1; // m/s
        static constexpr double kBaseSpeedWithVlx_ = 0.08; // m/s

        static constexpr double kBaseSpeedDownRamp_ = 0.15; // m/s


        static constexpr uint8_t kMaxMovements_ = 4;

        // TODO: Improve the name of this variable
        static constexpr double kMinWallDistance = 0.08; // 6 cm
        
        const double timeToTurn_ = 5000;

        static constexpr double kMaxErrorOrientation = 1.0; 

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

        unsigned long kTimeAfterRamp = 300; // 1400

        unsigned long kTimeAfterDownRamp = 1000; 

        bool downRamp_ = false;

        double currentDistance_ = 0;
        double targetDistance_ = 0;
        double distancePrev_ = 0;

        double crashDistance_ = 0;

        double timePrev_ = 0;

        double timePrevTurn_ = 0;

        VlxID vlxDirections[kNumberOfVlx] = {VlxID::kFrontRight, VlxID::kBack, VlxID::kLeft, VlxID::kRight, VlxID::kFrontLeft};

        double sampleTime_ = 100;

        static constexpr uint8_t kNumberOfWheels = 4;

        Motor motor[kNumberOfWheels];

        VLX vlx[kNumberOfVlx];

        double wallDistances[kNumberOfVlx];

        double normalizedDistances[3] = {0.05, 0.35, 0.65};

        static constexpr double kMToCenter_ = 0.05; // 5 cm

        // TODO: Improve the name of this variable
        static constexpr double kWallDistance = 0.18; // 18 cm

        static constexpr uint8_t kMaxDistanceErrorInCm = 1; // 1 cm

        static constexpr double kMaxDistanceErrorInMeters = 0.01; // 1 cm

        static constexpr double kMaxOrientationError = 0.9;

        static constexpr double kMinRampOrientation = 10.0;

        static constexpr double kMaxDistanceError = 0.01;

        static constexpr long long kOneSecInMs = 1000;

        static constexpr double kOneTileDistance = 0.3; // 0.228 m

        static constexpr double kMoveOneTileDistance = 0.30; // 0.228 m

        static constexpr long long kTileDirections = 4;

        double targetOrientation_ = 0;

        static constexpr double kUnreachableDistance = 0.85;

        static constexpr double kUnreachableDistanceInCm = 85;

        static constexpr double kWeightEncoders = 0.0;

        static constexpr double kWeightVlx = 1.0;

        static constexpr double kWeightBNO = 0.4;

        static constexpr double kWeightVLX = 0.6;

        int counterMovements_ = 0;

        bool encodersReset_ = false;

        bool inResetRoutine_ = false;

        constexpr static double speedOffset = 0.02;

        const double kOneCmInM = 0.01;

        PID pidDummy_;
        PID pidForward_;
        PID pidBackward_;
        PID pidTurn_;
        PID pidWallAlignment_;
        PID pidVLX_;
        PID pidBNO_;


        double vlxDistanceTraveled_;

        double distanceToCenter_;

        bool inCollision_ = false;

        constexpr static double kPForward = 0.07; // 0.09
        constexpr static double kIForward = 0.01;
        constexpr static double kDForward = 0.00;

        constexpr static double kPBackward = 0.02;
        constexpr static double kIBackward = 0.0;
        constexpr static double kDBackward = 0.0;

        constexpr static double kPTurn = 0.00005;
        double kITurn = 0.00000; // 0.00050
        constexpr static double kDTurn = 0.00019;

        constexpr static double kPDownRamp = 0.05;

        constexpr static double kPDistance = 0.05; // 0.20
        constexpr static double kIDistance = 0.00; // 0.00
        constexpr static double kDDistance = 0.082; // 0.082

        constexpr static double kPVLX = 0.63; // 0.63
        constexpr static double kIVLX = 0.00; // 0.00   
        constexpr static double kDVLX = 0.0; // 0.07

        constexpr static double kPBNO = 0.07; 
        constexpr static double kIBNO = 0.00;  
        constexpr static double kDBNO = 0.00;  

        constexpr static double kMaxErrorSum{4000};
        constexpr static double kMinOutput{0.003};
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
        static constexpr char kCheckpointColor = 'C';
        static constexpr double kHorizontalAngleError = 10;

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
            {20, 120, 30, 90, 20, 79},
            {85, 150, 80, 200, 120, 220}
        };

        bool blackTile_ = false;
        bool blueTile_ = false;
        bool checkpointTile_ = false;

        bool swithcVlx_ = false;

        bool leftVlx_ = true;

        bool turning_ = false;

        bool finishedMovement_ = false;

        static constexpr int kFiveSeconds_ = 5000;
        static constexpr int kTwoSeconds_ = 2000;

        // Dispenser stuff.
        // const char kCheckpointSerialCode = -1;
        const char kHarmedSerialCodeLeft = 'h';
        const char kHarmedSerialCodeRight = 'H';
        const char kStableSerialCodeLeft = 's';
        const char kStableSerialCodeRight = 'S';
        const char kUnharmedSerialCodeLeft = 'u';
        const char kUnharmedSerialCodeRight = 'U';
        const char kNoVictimSerialCode = 'm';
        const int kHalfSecond = 500;
        char victim = 'm';
        bool hasReceivedSerial = true;
        int kSendRequestCode = 1;
        int kResetSerialCode = 2;
        Servo myservo;
        static constexpr uint8_t initialAngle = 100;
        static constexpr uint8_t rightAngle = initialAngle + 37;
        static constexpr uint8_t leftAngle = initialAngle - 39;
        bool victimFound = false;
        u_int8_t rightStock = 6;
        u_int8_t leftStock = 6;

        Adafruit_SSD1306 display;

        bool lackOfProgress_ = false;

        bool alreadyInCollision_ = false;

        bool inRightCollision_ = false;

        bool inLeftCollision_ = false;

        unsigned long correctionTime_ = 800;

        double speedInCorrection_ = 0.10;

        uint8_t pwmRoutine_ = 120;

        bool detectedVictimInTurn_ = false;

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

        void moveMotorsInADirection(double targetOrientation, bool moveForward, bool inRamp = false);

        void setupVlx(const VlxID vlxId);

        void getAllWallsDistances(double wallDistances[kNumberOfVlx]);

        uint8_t checkWallsDistances();

        bool checkWallsDistances(const TileDirection targetTileDirection, const double currentOrientation);

        double getDistanceToCenter();

        double getWallDistance(const VlxID vlxId);

        void goForward(const double targetOrientation, const bool hasVictim);

        void goBackward(const double targetOrientation);

        void turnLeft(const double targetOrientation, const bool hasVictim);

        void turnRight(const double targetOrientation, const bool hasVictim);

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

        bool checkForCrashAndCorrect(bool crashLeft, bool crashRight, double currentOrientation, bool useWallDistance);
        
        void printEncoderTics();

        void resetSerial();

        void calibrateColors();

        void checkForLackOfProgress();

        bool getLackOfProgress();

        void resetLackOfProgress();

        void checkForBumper(); // TODO: Implement this function.

        bool onFlatGround(); // Not used.

        void printColorRanges();

        void maybeGoBackwards(const double currentOrientation);

        void weightPID(const double targetOrientation, const double currentOrientation, const double targetDistance, const double currentDistanceLeft, const double currentDistanceRight, double& speedLeft, double& speedRight);

        void testVlx();
};
#endif