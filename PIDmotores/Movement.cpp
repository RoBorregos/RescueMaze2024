#include "Movement.h"
#include "Pins.h"
#include "CustomSerial.h"
#include "VLX.h"

#define DEBUG_MOVEMENT 0
#define DEBUG_OFFLINE_MOVEMENT 0

#include <WiFi.h>
#include <WiFiUdp.h>

#if DEBUG_OFFLINE_MOVEMENT
const char* ssid = "RoBorregos2";
const char* password = "RoBorregos2024";
const char* udpServerIP = "192.168.0.123"; // Replace with your Python script's IP address
const int udpServerPort = 1;

WiFiUDP udp; 
#endif



Movement::Movement() {
    this->prevTimeTraveled_ = millis();
    this->motor[kNumberOfWheels];
    this->pidDummy_.setTunnings(0, 0, 0, 0, 0, 0, 0, 0, 0);
    this->pidForward_.setTunnings(kPForward, kIForward, kDForward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidBackward_.setTunnings(kPBackward, kIBackward, kDBackward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidTurn_.setTunnings(kPTurn, kITurn, kDTurn, kTurnMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedTurn_, kMaxOrientationError);
    this->pidWallAlignment_.setTunnings(kPDistance, kIDistance, kDDistance, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxDistanceError);
}

void Movement::setup() {
    this->prevTimeTraveled_ = millis();
    // WARNING? : The pidDummy_ doesn't work apparently the first pid won't work so by first writing an unused pid it will work with all the pids
    this->pidDummy_.setTunnings(kPForward, kIForward, kDForward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidForward_.setTunnings(kPForward, kIForward, kDForward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidBackward_.setTunnings(kPBackward, kIBackward, kDBackward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidTurn_.setTunnings(kPTurn, kITurn, kDTurn, kTurnMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedTurn_, kMaxOrientationError);
    this->pidWallAlignment_.setTunnings(kPDistance, kIDistance, kDDistance, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxDistanceError);

    #if DEBUG_OFFLINE_MOVEMENT
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(kOneSecInMs);
        // customPrintln("Connecting to WiFi...");
    }
    // customPrintln("Connected to WiFi");

    udp.begin(udpServerPort);
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("Connected to WiFi");
    udp.endPacket();
    #endif

    setupInternal(MotorID::kFrontLeft);
    setupInternal(MotorID::kFrontRight);
    setupInternal(MotorID::kBackLeft);
    setupInternal(MotorID::kBackRight);
    bno_.setupBNO();
    Wire.begin();
    // TODO: Make a loop in which it setup the VLXs
    setupVlx(VlxID::kFrontRight);
    setupVlx(VlxID::kLeft);
    setupVlx(VlxID::kBack);
    setupVlx(VlxID::kRight);
    setupVlx(VlxID::kFrontLeft); 

    setupLimitSwitch(LimitSwitchID::kLeft);
    setupLimitSwitch(LimitSwitchID::kRight);

    myservo.attach(Pins::servoPin);

    // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
    this->display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);    

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
        // customPrintln(F("SSD1306 allocation failed"));
        for(;;);
    }

    setupTCS();

    pinMode(Pins::buttonPin, INPUT_PULLUP);

    sendSerialRequest();
}

void Movement::setupInternal(const MotorID motorId) {
    const uint8_t index = static_cast<uint8_t>(motorId);
    motor[index].setupMotor(
        Pins::pwmPin[index],
        Pins::digitalOne[index],
        Pins::digitalTwo[index],
        Pins::encoderA[index],
        motorId);
}

void Movement::setupVlx(const VlxID vlxId) {
    const uint8_t index = static_cast<uint8_t>(vlxId);
    vlx[index].setMux(Pins::vlxPins[index]);
    vlx[index].init();
}

void Movement::setupLimitSwitch(const LimitSwitchID limitSwitchId) {
    const uint8_t index = static_cast<uint8_t>(limitSwitchId);
    limitSwitch_[index].initLimitSwitch(Pins::limitSwitchPins[index]);
}

void Movement::setupTCS() {
    tcs_.setMux(Pins::tcsPins[0]);
    tcs_.setPrecision(kPrecision);
    tcs_.init(kColors, kColorAmount, kColorList, kColorThresholds);
}

void Movement::stopMotors() {
    for (uint8_t i = 0; i < kNumberOfWheels; ++i) {
        motor[i].motorStop();
    }
}

void Movement::setSpeed(const double speed) { // Speed in meters per second
    for (uint8_t i = 0; i < kNumberOfWheels; ++i) {
        motor[i].constantSpeed(speed, MotorState::kForward);
    }
}

void Movement::setPwmsAndDirections(const uint8_t pwms[kNumberOfWheels], const MotorState directions[kNumberOfWheels]) {
    for (uint8_t i = 0; i < kNumberOfWheels; ++i) {
        motor[i].setPwmAndDirection(pwms[i], directions[i]);
    }
}

void Movement::setSpeedsAndDirections(const double speeds[kNumberOfWheels], const MotorState directions[kNumberOfWheels]) {
    for (uint8_t i = 0; i < kNumberOfWheels; ++i) {
        // customPrint(speeds[i]);
        motor[i].setSpeedAndDirection(speeds[i], directions[i]);
    }
}

void Movement::setMotorsDirections(const MovementState state, MotorState directions[kNumberOfWheels]) {
    const uint8_t frontLeftIndex = static_cast<uint8_t>(MotorID::kFrontLeft);
    const uint8_t frontRightIndex = static_cast<uint8_t>(MotorID::kFrontRight);
    const uint8_t backLeftIndex = static_cast<uint8_t>(MotorID::kBackLeft);
    const uint8_t backRightIndex = static_cast<uint8_t>(MotorID::kBackRight);
    switch (state)
    {
    case (MovementState::kForward):{
            directions[frontLeftIndex] = MotorState::kForward;
            directions[backLeftIndex] = MotorState::kForward;
            directions[frontRightIndex] = MotorState::kForward;
            directions[backRightIndex] = MotorState::kForward;
        break;
    }
    case (MovementState::kBackward):{
            directions[frontLeftIndex] = MotorState::kBackward;
            directions[backLeftIndex] = MotorState::kBackward;
            directions[frontRightIndex] = MotorState::kBackward;
            directions[backRightIndex] = MotorState::kBackward;
        break;
    }
    case (MovementState::kTurnLeft):{
            directions[frontLeftIndex] = MotorState::kBackward;
            directions[backLeftIndex] = MotorState::kBackward;
            directions[frontRightIndex] = MotorState::kForward;
            directions[backRightIndex] = MotorState::kForward;
        break;
    }
    case (MovementState::kTurnRight):{
            directions[frontLeftIndex] = MotorState::kForward;
            directions[backLeftIndex] = MotorState::kForward;
            directions[frontRightIndex] = MotorState::kBackward;
            directions[backRightIndex] = MotorState::kBackward;
        break;
    }
    default:
        break;
    }
}

void Movement::moveMotorsInADirection(double targetOrientation, bool moveForward, bool inRamp){
    const unsigned long timeDiff = millis() - timePrev_;
    double currentOrientation = bno_.getOrientationX();
    if (timeDiff < sampleTime_) {
        return;
    }

    double speeds[kNumberOfWheels];
    MotorState directions[kNumberOfWheels]; 

    double speedLeft = 0;
    double speedRight = 0;

    const uint8_t frontLeftIndex = static_cast<uint8_t>(MotorID::kFrontLeft);
    const uint8_t frontRightIndex = static_cast<uint8_t>(MotorID::kFrontRight);
    const uint8_t backLeftIndex = static_cast<uint8_t>(MotorID::kBackLeft);
    const uint8_t backRightIndex = static_cast<uint8_t>(MotorID::kBackRight);

    if (moveForward) {
        if (inRamp) {
            pidWallAlignment_.computeDistance(0.11, vlx[static_cast<uint8_t>(VlxID::kLeft)].getRawDistance(), speedLeft, speedRight);
            /* udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("SpeedLeft:" + String(speedLeft));
            udp.print(" ");
            udp.print("SpeedRight:" + String(speedRight));
            udp.endPacket(); */
        } else {
            pidForward_.computeStraight(targetOrientation, currentOrientation, speedLeft, speedRight);
        }
        speeds[frontLeftIndex] = speedLeft;
        speeds[backLeftIndex] = speedLeft;
        speeds[frontRightIndex] = speedRight;
        speeds[backRightIndex] = speedRight;
        #if DEBUG_MOVEMENT
        customPrintln("SpeedLeft:" + String(speedLeft));
        customPrintln("SpeedRight:" + String(speedRight));
        #endif
        setMotorsDirections(MovementState::kForward, directions);
    } else {
        pidBackward_.computeStraight(targetOrientation, currentOrientation, speedLeft, speedRight);
        speeds[frontLeftIndex] = speedRight;
        speeds[backLeftIndex] = speedRight;
        speeds[frontRightIndex] = speedLeft;
        speeds[backRightIndex] = speedLeft;

        setMotorsDirections(MovementState::kBackward, directions);
    }

    setSpeedsAndDirections(speeds, directions);

    /* udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("SpeedLeft:" + String(speedLeft));
    udp.print(" ");
    udp.print("SpeedRight:" + String(speedRight));
    udp.endPacket(); */

    timePrev_ = millis();
}

void Movement::getAllWallsDistances(double wallDistances_[kNumberOfVlx]) {
    for (const auto& vlxDirection : vlxDirections) {
        const uint8_t index = static_cast<uint8_t>(vlxDirection);
        wallDistances_[index] = getWallDistance(vlxDirection);
        //customPrint("VLX:" + String(index)) ;
        //customPrintln("WallDistance:" + String(wallDistances_[index]));
    }

    #if DEBUG_MOVEMENT
    customPrintln("FrontRight:" + String(wallDistances[static_cast<uint8_t>(VlxID::kFrontRight)]));
    customPrintln("Back:" + String(wallDistances[static_cast<uint8_t>(VlxID::kBack)]));
    customPrintln("Left:" + String(wallDistances[static_cast<uint8_t>(VlxID::kLeft)]));
    customPrintln("Right:" + String(wallDistances[static_cast<uint8_t>(VlxID::kRight)]));
    customPrintln("FrontLeft:" + String(wallDistances[static_cast<uint8_t>(VlxID::kFrontLeft)]));
    #endif
}

int8_t Movement::getIndexFromArray(const int value, const int array[], const uint8_t arraySize) {
    for (uint8_t i = 0; i < arraySize; ++i) {
        const int currentValue = array[i];
        if (currentValue == value) {
            return i;
        } 
    }

    return -1;
}

bool Movement::checkWallsDistances(const TileDirection targetTileDirection, const double currentOrientation) {
    const int8_t orientationIndex = getIndexFromArray(currentOrientation, kTargetOrientations, kNumberOfTargetOrientations);

    if (orientationIndex >= kNumberOfTargetOrientations || orientationIndex < kOffArray) {
        #if DEBUG_MOVEMENT
        customPrintln("Orientation not found in the array.");
        #endif
        // customPrintln("Orientation not found in the array.");
        return false;
    }

    const uint8_t vlxIndex = (static_cast<uint8_t>(targetTileDirection) + orientationIndex) % kTileDirections;
    const VlxID vlxID = static_cast<VlxID>(vlxIndex);

    // customPrintln("WALL?: " + String(getWallDistance(vlxID)) + " < " + "0.15" + " = " + String(getWallDistance(vlxID) < 0.15));
    return getWallDistance(vlxID) < 0.17;
}

uint8_t Movement::checkWallsDistances() {
    // 00000
    // FBLRF
    uint8_t wallDetected = 0;
    for (uint8_t i = 0; i < kNumberOfVlx; ++i) {
        wallDetected |= (wallDistances[i] < kWallDistance ? 1 : 0) << i;
    }

    return wallDetected;
}

double Movement::getWallDistance(const VlxID vlxId) {
    //customPrintln("wallDistances: " + String(vlx[static_cast<uint8_t>(vlxId)].getDistance()));
    return vlx[static_cast<uint8_t>(vlxId)].getRawDistance();
    // return wallDistances[static_cast<uint8_t>(vlxId)];
}

void Movement::goForward(const double targetOrientation, const bool hasVictim) {
    victimFound = hasVictim;
    moveMotors(MovementState::kForward, targetOrientation, kMoveOneTileDistance);
}

void Movement::goBackward(const double targetOrientation) {
    moveMotors(MovementState::kBackward, targetOrientation, kMoveOneTileDistance);
}

void Movement::turnLeft(const double targetOrientation) {
    turning_ = true;
    moveMotors(MovementState::kTurnLeft, targetOrientation, 0);
}

void Movement::turnRight(const double targetOrientation) {
    turning_ = true;
    moveMotors(MovementState::kTurnRight, targetOrientation, 0);
}

void Movement::rampMovement(const double targetOrientation) {
    moveMotors(MovementState::kRamp, targetOrientation, 0);
}

// TODO: Clean this function
void Movement::moveMotors(const MovementState state, const double targetOrientation, double targetDistance, bool useWallDistance, bool center) { 
    // 321
    lackOfProgress_ = false;
    double speeds[kNumberOfWheels];
    MotorState directions[kNumberOfWheels]; 
    double currentOrientation = bno_.getOrientationX(); 
    double speedLeft = 0;
    double speedRight = 0;
    bool turnLeft = false;
    const uint8_t frontLeftIndex = static_cast<uint8_t>(MotorID::kFrontLeft);
    const uint8_t frontRightIndex = static_cast<uint8_t>(MotorID::kFrontRight);
    const uint8_t backLeftIndex = static_cast<uint8_t>(MotorID::kBackLeft);
    const uint8_t backRightIndex = static_cast<uint8_t>(MotorID::kBackRight);

    const uint8_t leftLimitSwitch = static_cast<uint8_t>(LimitSwitchID::kLeft);
    const uint8_t rightLimitSwitch = static_cast<uint8_t>(LimitSwitchID::kRight);

    bool crashRight = false;
    bool crashLeft = false;
    bool rampDetected = false;

    uint8_t vlxId = static_cast<uint8_t>(VlxID::kFrontLeft);

    const double initialFrontWallDistance = vlx[vlxId].getRawDistance();
    const double initialBackWallDistance = vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance();
    bool moveForward = false;

    ++counterMovements_;
    
    // customPrintln("CounterMovements:" + String(counterMovements_));
    
    prevTimeTraveled_ = millis();

    switch (state)
    {
        case (MovementState::kStop): {
            stopMotors();
            break;
        }
        case (MovementState::kForward): {
            resetSerial();
            moveForward = true;
            currentState_ = MovementState::kForward;
            blackTile_ = false;
            blueTile_ = false;
            checkpointTile_ = false;
            finishedMovement_ = false;
            victimFound = false; // Testing.

            // customPrintln("Moving forward");
            // customPrintln("TargetDistance:" + String(targetDistance));

            double frontWallDistance = initialFrontWallDistance;
            double backWallDistance = initialBackWallDistance;
            
            // TODO: improve the way to give a targetDistance with vlx
            // targetDistance = getRealTargetDistance(targetDistance);

            /* udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("targetDistance:" + String(targetDistance));
            udp.endPacket(); */
            #if DEBUG_OFFLINE_MOVEMENT
            #endif

            while (hasTraveledDistanceWithSpeed(targetDistance) == false  && lackOfProgress_ == false) {
                checkForLackOfProgress();
                
                if (wasBlackTile()) {
                    return;
                }
                checkColors(targetOrientation);

                frontWallDistance = vlx[vlxId].getRawDistance();
                crashLeft = limitSwitch_[leftLimitSwitch].getState();
                crashRight = limitSwitch_[rightLimitSwitch].getState();

                bool result = checkForCrashAndCorrect(crashLeft, crashRight, currentOrientation, useWallDistance);
                if (!result) {
                    stopMotors();
                    break;
                }

                // Checking serial.
                if (victimFound == false) {
                    if (hasReceivedSerial == true) {
                        screenPrint("request sent");
                        sendSerialRequest();
                    }
                    // if (vlx[static_cast<uint8_t>(VlxID::kFrontRight)].getRawDistance() < kWallDistance) {
                    checkSerial(currentOrientation);
                    // }
                }

                // customPrintln("Color:" + String(getTCSInfo()));
                // customPrintln("blackTile" + String(blackTile_));
                #if DEBUG_MOVEMENT
                #endif


                #if DEBUG_OFFLINE_MOVEMENT
                udp.beginPacket(udpServerIP, udpServerPort);
                udp.print("targetOrientation" + String(targetOrientation));
                udp.print(" ");
                udp.print("CurrentOreintation" + String(currentOrientation));
                udp.print(" ");
                udp.print("CurrentDistance" + String(targetDistance));
                udp.print(" ");
                udp.endPacket();
                #endif
                #if DEBUG_MOVEMENT
                // customPrintln("Rampa: " + String(isRamp()));
                #endif
                moveMotorsInADirection(targetOrientation, true);

            }

            stopMotors();


            // udp.beginPacket(udpServerIP, udpServerPort);
            // udp.print("Antes de entrar a vlx");
            // udp.endPacket();

            // Center in tile with vlx.
            if (inResetRoutine_ == false || useWallDistance == false) {
               
                frontWallDistance = vlx[vlxId].getRawDistance();
                backWallDistance = vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance();
                // udp.beginPacket(udpServerIP, udpServerPort);
                // udp.print("ENTRA EN IF");
                // udp.endPacket();

                while (static_cast<uint8_t>(abs(weightMovementVLX(backWallDistance, 
                                             frontWallDistance, 
                                             initialBackWallDistance, 
                                             initialFrontWallDistance, 
                                             moveForward, 
                                             targetDistance) - (targetDistance * kMToCm))) >= kMaxDistanceErrorInCm  && lackOfProgress_ == false) {
                    
                    checkForLackOfProgress();

                    if (wasBlackTile()) {
                        return;
                    }
                    checkColors(targetOrientation);

                    // Checking serial.
                    if (victimFound == false) {
                        if (hasReceivedSerial == true) {
                            screenPrint("request sent on vlx");
                            sendSerialRequest();
                        }
                        // if (vlx[static_cast<uint8_t>(VlxID::kFrontRight)].getRawDistance() < kWallDistance) {
                        checkSerial(currentOrientation);
                        // }
                    }

                    pidBackward_.setBaseSpeed(kBaseSpeedForwardReset_);
                    pidForward_.setBaseSpeed(kBaseSpeedForwardReset_);
                    
                    // customPrintln("vlxFrontLeft:" + String(frontWallDistance));
                    // customPrintln("vlxBack:" + String(backWallDistance));
                
                    if (!moveForward || backWallDistance < kUnreachableDistance) {
                        backWallDistance = vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance();
                    }
                    if (moveForward || frontWallDistance < kUnreachableDistance) {
                        frontWallDistance = vlx[vlxId].getRawDistance();   
                    }

                    
                    if (moveForward){
                        crashLeft = limitSwitch_[leftLimitSwitch].getState();
                        crashRight = limitSwitch_[rightLimitSwitch].getState();
                    }
                    #if DEBUG_OFFLINE_MOVEMENT
                    udp.beginPacket(udpServerIP, udpServerPort);
                    udp.print("crashLeft:" + String(crashLeft));
                    udp.print(" ");
                    udp.print("crashRight:" + String(crashRight));
                    udp.endPacket();
                    #endif
                    bool result = checkForCrashAndCorrect(crashLeft, crashRight, currentOrientation, useWallDistance);
                    if (!result) {
                        stopMotors();
                        break;
                    }

                    // if (moveForward && frontWallDistance <= kMinWallDistance) {
                    //     stopMotors();
                    //     break;
                    // }

                    // TODO: Only check colors when it is moving forward
                    // checkColors(currentOrientation);
                    
                    #if DEBUG_MOVEMENT
                    customPrintln("Color:" + String(getTCSInfo()));
                    customPrintln("blackTile" + String(blackTile_));
                    #endif

                    #if DEBUG_OFFLINE_MOVEMENT
                    udp.beginPacket(udpServerIP, udpServerPort);
                    udp.print("targetOrientation" + String(targetOrientation));
                    udp.print(" ");
                    udp.print("CurrentOreintation" + String(currentOrientation));
                    udp.print(" ");
                    udp.print("CurrentDistance" + String(targetDistance));
                    udp.endPacket();
                    #endif

                    moveMotorsInADirection(targetOrientation, moveForward);

                    pidBackward_.setBaseSpeed(kBaseSpeedForward_);
                    pidForward_.setBaseSpeed(kBaseSpeedForward_);
                }
            }

            finishedMovement_ = true;
            swithcVlx_ = false;
            turning_ = false;
            checkColors(targetOrientation);
            
            allDistanceTraveled_ = 0;
            stopMotors();

            screenPrint("Finished movement");
            // delay(kOneSecInMs);
            break;
        }
        case (MovementState::kBackward): {
            moveForward = false;
            if (inResetRoutine_) {
                resetDistanceTraveled_ = true;
            }

            while (hasTraveledDistanceWithSpeed(targetDistance) == false  && lackOfProgress_ == false) {
                checkForLackOfProgress();
                #if DEBUG_OFFLINE_MOVEMENT
                udp.beginPacket(udpServerIP, udpServerPort);
                udp.print("--------------------------------------------------------------");
                udp.endPacket();
                #endif
                moveMotorsInADirection(targetOrientation, moveForward);
            }

            allDistanceTraveled_ = 0;

            stopMotors();
            
            return;
        }
        // TODO: change MotorStarte of turnRigth and left to make an oneself motorState and with that I mean turn 
        case (MovementState::kTurnLeft): {
            resetSerial();
            currentState_ = MovementState::kTurnLeft;
            turnMotors(targetOrientation, targetDistance, currentOrientation);

            break;
        }
        case (MovementState::kTurnRight): {
            resetSerial();
            currentState_ = MovementState::kTurnRight;
            turnMotors(targetOrientation, targetDistance, currentOrientation);

            break;
        }
        case (MovementState::kRamp): {
            #if DEBUG_MOVEMENT
            customPrintln("kRamp");
            #endif
            rampDetected = isRamp();
            while (rampDetected) {
                moveMotorsInADirection(targetOrientation, true, true);
                rampDetected = isRamp();
            }
            
            const unsigned long timePrevRamp = millis();
            unsigned long timeDiff = millis() - timePrevRamp;
            // customPrintln("timePrevRamp:" + String(timePrevRamp));
            // TODO: Maybe delete this while loop
            while (timeDiff < kTimeAfterRamp) {
                // customPrintln("TimeDiff:" + String(timeDiff));
                timeDiff = millis() - timePrevRamp;
                moveMotorsInADirection(targetOrientation, true);
            }
            
            stopMotors();

            #if DEBUG_MOVEMENT
            customPrintln("Ramp finished");
            customPrintln("hasWallInFront:" + String(hasWallInFront()));
            customPrintln("vlxFrontLeft: " + String(vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance()));
            #endif
            
            moveMotors(MovementState::kForward, targetOrientation, kTileLengthInMeters, true, false);
            

            break;
        }
    }

    maybeResetWithBackWall(targetOrientation, currentOrientation);
}


void Movement::correctionAfterCrash(const bool crashLeft, double currentOrientation, bool useWallDistance) {
    useWallDistance = false;
    saveLastState(getCurrentState(), currentOrientation);
    moveMotors(MovementState::kStop, 0, 0);
    if (crashLeft == false) {
        #if DEBUG_MOVEMENT
        customPrintln("Crash right--------");
        #endif
                                                         // currentOrientation - crashDeltaOrientation_
        moveMotors(MovementState::kBackward, getOrientation(currentOrientation), crashDeltaDistance_ , false, false);
        moveMotors(MovementState::kTurnLeft, getOrientation(currentOrientation + crashDeltaOrientation_), 0);
    } else {
        #if DEBUG_MOVEMENT
        customPrintln("Crash left--------");
        #endif
                                                         // currentOrientation + crashDeltaOrientation_
        moveMotors(MovementState::kBackward, getOrientation(currentOrientation), crashDeltaDistance_ , false, false);
        moveMotors(MovementState::kTurnRight, getOrientation(currentOrientation - crashDeltaOrientation_), 0);
    }
    retrieveLastState();
}

double Movement::getOrientation(const double orientation) {
    if (orientation < 0) {
        return orientation + 360;
    } else if (orientation > 360) {
        return orientation - 360;
    } else {
        return orientation;
    }
}

void Movement::saveLastState(const MovementState state, double &targetOrientation) {
    crashDistance_ = allDistanceTraveled_;
    targetOrientation_ = targetOrientation;
    allDistanceTraveled_ = 0;
    lastState_ = state;
    
    #if DEBUG_MOVEMENT
    customPrintln("CrashDistance:" + String(crashDistance_));
    customPrintln("TargetOrientation:" + String(targetOrientation_));
    #endif
}

void Movement::retrieveLastState() {
    allDistanceTraveled_ = crashDistance_ - crashDeltaDistance_;
    currentState_ = lastState_;
}

void Movement::turnMotors(const double targetOrientation, const double targetDistance, double &currentOrientation){
    double speeds[kNumberOfWheels];
    MotorState directions[kNumberOfWheels]; 
    double speed = 0;
    bool turnLeft = false;

    #if DEBUG_OFFLINE_MOVEMENT
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("TargetOrientation:" + String(targetOrientation));
    udp.endPacket();
    #endif
    timePrevTurn_ = millis();
    while (abs(pidTurn_.computeErrorOrientation(targetOrientation, currentOrientation)) > kMaxOrientationError  && lackOfProgress_ == false) {
        checkForLackOfProgress();
        // Checking serial.
        if (victimFound == false) {
            if (hasReceivedSerial == true) {
                screenPrint("request sent");
                sendSerialRequest();
            }
            // if (vlx[static_cast<uint8_t>(VlxID::kRight)].getRawDistance() < kWallDistance) {
            checkSerial(currentOrientation);
            // }
        }
        
        maybeGoBackwards(currentOrientation);

        #if DEBUG_OFFLINE_MOVEMENT
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("CurrentOrientation:" + String(currentOrientation));
        udp.print(" ");
        udp.print("TargetOrientation:" + String(targetOrientation));
        udp.endPacket();
        #endif

        #if DEBUG_MOVEMENT
        customPrintln("ErrorOrientation:" + String(pidTurn_.computeErrorOrientation(targetOrientation, currentOrientation)));
        #endif
        const unsigned long timeDiff = millis() - timePrev_;
        if (timeDiff < sampleTime_) {
            currentOrientation = bno_.getOrientationX();
            continue;
        }

        pidTurn_.computeTurn(targetOrientation, currentOrientation, speed, turnLeft);

        #if DEBUG_OFFLINE_MOVEMENT
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("Speed:" + String(speed));
        udp.endPacket();
        #endif


        if (turnLeft) {
            setMotorsDirections(MovementState::kTurnLeft, directions); 
        } else {
            setMotorsDirections(MovementState::kTurnRight, directions); 
        }
        
        for (uint8_t i = 0; i < kNumberOfWheels; ++i) {
            speeds[i] = speed;
        }

        setSpeedsAndDirections(speeds, directions);

        timePrev_ = millis();
        
    }
    stopMotors();
    
}

MovementState Movement::getCurrentState() {
    return currentState_;
}

void Movement::updateTics(MotorID motorId) {
    const uint8_t index = static_cast<uint8_t>(motorId);
    if (index >= 0 && index < kNumberOfWheels) {
        motor[index].deltaTics(1);
        if (motor[index].getCurrentState() == MotorState::kForward){
            motor[index].deltaTotalTics(1);
        } else if (motor[index].getCurrentState() == MotorState::kBackward){
            motor[index].deltaTotalTics(-1);
        }
    }
}

double Movement::getBackLeftSpeed() {
    return motor[static_cast<uint8_t>(MotorID::kBackLeft)].getSpeed();
}

double Movement::getFrontLeftSpeed() {
    return motor[static_cast<uint8_t>(MotorID::kFrontLeft)].getSpeed();
}

double Movement::getBackRightSpeed() {
    return motor[static_cast<uint8_t>(MotorID::kBackRight)].getSpeed();
}

double Movement::getFrontRightSpeed() {
    return motor[static_cast<uint8_t>(MotorID::kFrontRight)].getSpeed();
}

uint8_t Movement::getOrientation(const compass currentOrientation) {
    switch (currentOrientation) {
        case (compass::kNorth): {
            return 0;
        }
        case (compass::kEast): {
            return 90;
        }
        case (compass::kSouth): {
            return 180;
        }
        case (compass::kWest): {
            return 270;
        }
        default: {
            return 0;
        }
    }
}

bool Movement::hasTraveledDistanceWithSpeed(const double distance) {
    const unsigned long timeTraveled_ = millis() - prevTimeTraveled_;
    //customPrintln("TimeTraveled:" + String(timeTraveled_));
    if (timeTraveled_ < kSampleTimeTraveled) {
        return false;
    }
    // TODO: Improve the way to calculate the average speed
    const double averageSpeed = (motor[static_cast<uint8_t>(MotorID::kBackLeft)].getSpeed() +
                                 motor[static_cast<uint8_t>(MotorID::kBackRight)].getSpeed() +
                                 motor[static_cast<uint8_t>(MotorID::kFrontLeft)].getSpeed() +
                                 motor[static_cast<uint8_t>(MotorID::kFrontRight)].getSpeed()) / kNumberOfWheels;
    
    const double distanceTraveled = averageSpeed * (timeTraveled_ / (double)kOneSecInMs);

    //customPrintln("averageSpeed: " + String(averageSpeed));
    //customPrintln("timeTraveled: " + String(timeTraveled_));
    //customPrintln("kOneInSecMs: " + String(kOneSecInMs));
    allDistanceTraveled_ += distanceTraveled;
    //customPrintln("distanceTraveled: " + String(distanceTraveled));

    //customPrintln("allDistanceTraveled___:" + String(allDistanceTraveled_));
    prevTimeTraveled_ = millis();

    
    #if DEBUG_OFFLINE_MOVEMENT
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("DistanceTraveled:" + String(allDistanceTraveled_));
    udp.endPacket();
    #endif

    // Reset the distance traveled only for backwardMovement when it has a wall behind and it has a counterMovements_ >= 4
    if (allDistanceTraveled_ >= distance) {
        if (resetDistanceTraveled_ == true) {
            allDistanceTraveled_ = 0;
            resetDistanceTraveled_ = false;
        }
        #if DEBUG_OFFLINE_MOVEMENT
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("totalDistanceTraveled:" + String(allDistanceTraveled_));
        udp.endPacket();
        #endif
        /* udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("TRUE");
        udp.endPacket(); */
        return true;
    }
    
    return false;
}

bool Movement::hasTraveledWallDistance(double targetDistance, double currentDistance) {
    const double distanceDiff = (targetDistance - currentDistance);
    return abs(distanceDiff) < kMaxDistanceErrorInCm;
}

void Movement::printTCS() {
    tcs_.printRGB();
    tcs_.getColor();
}

char Movement::getTCSInfo() {
    return tcs_.getColorWithThresholds();
} 

void Movement::rgbTCSClear() {
    tcs_.printRGBC();
}

char Movement::checkColors(const double targetOrientation) {
    const char color = tcs_.getColor();
    if (color == kBlackColor) {
        screenPrint("Black tile detected");
        blackTile_ = true;
        const double desiredDistance = allDistanceTraveled_;
        targetDistance_ = desiredDistance;
        allDistanceTraveled_ = 0;
        stopMotors();
        moveMotors(MovementState::kBackward, targetOrientation, targetDistance_);
        return color;
    } else if (color == kBlueColor && finishedMovement_ == true && !blueTile_) {
        screenPrint("Blue tile detected");
        blueTile_ = true;
        stopMotors();
        delay(kFiveSeconds_);
        return color;
    } else if (color == 'C' && finishedMovement_ == true && !checkpointTile_ && bno_.getOrientationY() < kHorizontalAngleError && bno_.getOrientationY() > -kHorizontalAngleError) { 
        screenPrint("Checkpoint tile detected");
        checkpointTile_ = true;
        stopMotors();
        delay(kOneSecInMs);
        return color;
    }
    return color;
}

bool Movement::isRamp() {
    const double currentOrientationY = bno_.getOrientationY();

    #if DEBUG_OFFLINE_MOVEMENT
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("OrientationY:" + String(currentOrientationY));
    udp.endPacket();
    #endif

    // customPrintln("OrientationY:" + String(currentOrientationY));
    #if DEBUG_MOVEMENT
    #endif
    if (currentOrientationY >= kMinRampOrientation || currentOrientationY <= -kMinRampOrientation) {
        #if DEBUG_MOVEMENT
        customPrintln("TRUE");
        #endif
        return true;
    }
    #ifndef DEBUG_MOVEMENT
    customPrintln("FALSE");
    #endif
    return false;
}

double Movement::weightMovement(const double currentDistanceBack, const double currentDistanceFront, const double initialVlxDistanceBack, const double initialVlxDistanceFront) {
    double vlxDistanceTraveled = 0;
    bool rampDetected = isRamp();

    if (rampDetected) {
        #if DEBUG_MOVEMENT
        customPrintln("Ramp detected");
        #endif
        return allDistanceTraveled_;
    }

    if (currentDistanceBack < currentDistanceFront) {
        if (initialVlxDistanceBack >= kUnreachableDistance || currentDistanceBack >= kUnreachableDistance) {
            #if DEBUG_OFFLINE_MOVEMENT
            udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("Unreachable");
            udp.print(" ");
            udp.print("initialVlxDistanceBack:" + String(initialVlxDistanceBack));
            udp.endPacket();
            #endif
            return allDistanceTraveled_;
        }
        vlxDistanceTraveled = currentDistanceBack - initialVlxDistanceBack;
        #if DEBUG_OFFLINE_MOVEMENT
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("Back");
        udp.print(" ");
        udp.print("currentDistanceBack:" + String(currentDistanceBack));
        udp.print(" ");
        udp.print("currentDistanceFront:" + String(currentDistanceFront));
        udp.print(" ");
        udp.print("initialVlxDistanceBack:" + String(initialVlxDistanceBack));
        udp.print(" ");
        udp.print("vlxDistanceTraveled:" + String(vlxDistanceTraveled));
        udp.endPacket();
        #endif
    } else {
        if (initialVlxDistanceFront >= kUnreachableDistance || currentDistanceFront >= kUnreachableDistance) {
            #if DEBUG_OFFLINE_MOVEMENT
            udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("Unreachable");
            udp.print(" ");
            udp.print("initialVlxDistanceFront:" + String(initialVlxDistanceFront));
            udp.endPacket();
            #endif
            return allDistanceTraveled_;
        }
        vlxDistanceTraveled = initialVlxDistanceFront - currentDistanceFront;
        #if DEBUG_OFFLINE_MOVEMENT
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("Front");
        udp.print(" ");
        udp.print("currentDistanceBack:" + String(currentDistanceBack));
        udp.print(" ");
        udp.print("currentDistanceFront:" + String(currentDistanceFront));
        udp.print(" ");
        udp.print("initialVlxDistanceFront:" + String(initialVlxDistanceFront));
        udp.print(" ");
        udp.print("vlxDistanceTraveled:" + String(vlxDistanceTraveled));
        udp.endPacket();
        #endif
    }
    #if DEBUG_OFFLINE_MOVEMENT
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("allDistanceTraveled:" + String(allDistanceTraveled_));
    udp.print(" ");
    udp.print("vlxDistanceTraveled:" + String(vlxDistanceTraveled));
    udp.print(" ");
    udp.print("allMovement:" + String(allDistanceTraveled_ * kWeightEncoders + vlxDistanceTraveled * kWeightVlx));
    udp.endPacket();
    #endif
    const double weightMovement = allDistanceTraveled_ * kWeightEncoders + vlxDistanceTraveled * kWeightVlx;

    return (weightMovement * kMToCm);
    
}

double Movement::weightMovementVLX(const double currentDistanceBack, const double currentDistanceFront, const double initialVlxDistanceBack, const double initialVlxDistanceFront, bool& moveForward, double targetDistance) {
    double vlxDistanceTraveled = 0;
    bool rampDetected = isRamp();

    if (rampDetected) {
        #if DEBUG_MOVEMENT
        customPrintln("Ramp detected");
        #endif
        return targetDistance * kMToCm;
    }

    if (currentDistanceBack < currentDistanceFront) {
        if (initialVlxDistanceBack >= kUnreachableDistance || currentDistanceBack >= kUnreachableDistance) {
            // udp.beginPacket(udpServerIP, udpServerPort);
            // udp.print("currentDistanceBack:" + String(currentDistanceBack));
            // udp.print(" ");
            // udp.print("initialVlxDistanceBack:" + String(initialVlxDistanceBack));
            // udp.print("Unreachable BACK");
            // udp.endPacket();
            return targetDistance * kMToCm;
        }

        // udp.beginPacket(udpServerIP, udpServerPort);
        // udp.print("currentDistanceBack:" + String(currentDistanceBack));
        // udp.print(" ");
        // udp.print("initialVlxDistanceBack:" + String(initialVlxDistanceBack));
        // udp.print(" ");
        // udp.print("vlxDistanceTraveled BACK:" + String(currentDistanceBack - initialVlxDistanceBack)	);
        // udp.endPacket();

        vlxDistanceTraveled = currentDistanceBack - initialVlxDistanceBack;
       
    } else {
        if (initialVlxDistanceFront >= kUnreachableDistance || currentDistanceFront >= kUnreachableDistance) {
            // udp.beginPacket(udpServerIP, udpServerPort);
            // udp.print("currentDistanceFront:" + String(currentDistanceFront));
            // udp.print(" ");
            // udp.print("initialVlxDistanceFront:" + String(initialVlxDistanceFront));
            // udp.print(" ");
            // udp.print("Unreachable FRONT");
            // udp.endPacket();
            return targetDistance * kMToCm;
        }
        // udp.beginPacket(udpServerIP, udpServerPort);
        // udp.print("currentDistanceFront:" + String(currentDistanceFront));
        // udp.print(" ");
        // udp.print("initialVlxDistanceFront:" + String(initialVlxDistanceFront));
        // udp.endPacket();

        vlxDistanceTraveled = initialVlxDistanceFront - currentDistanceFront;
    }

    const uint8_t weightMovement = (vlxDistanceTraveled * kWeightVlx) * kMToCm;
    moveForward = static_cast<int8_t>((vlxDistanceTraveled - targetDistance) * kMToCm) < 0;
    // udp.beginPacket(udpServerIP, udpServerPort);
    // udp.print("currentDistanceBack:" + String(currentDistanceBack));
    // udp.print(" ");
    // udp.print("currentDistanceFront:" + String(currentDistanceFront));
    // udp.print(" ");
    // udp.print("weightMovement: " + String(weightMovement));
    // udp.print(" ");
    // udp.print("targetDistance: " + String(targetDistance));
    // udp.print(" ");
    // udp.print("moveForward: " + String(moveForward));
    // udp.endPacket();
    return weightMovement;

}

void Movement::updateDistanceToCenterInTile() {
    distanceToCenter_ = (kTileLength - kLengthOfRobot) / 2;
    distanceToCenter_ = distanceToCenter_ / kMToCm;

    #if DEBUG_OFFLINE_MOVEMENT
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("kTileLength:" + String(kTileLength));
    udp.print(" ");
    udp.print("kLengthOfRobot:" + String(kLengthOfRobot));
    udp.print(" ");
    udp.print("distanceToCenter__:" + String(distanceToCenter_));
    udp.endPacket();
    #endif
}

bool Movement::hasWallBehind() {
    #if DEBUG_MOVEMENT
    customPrintln("BackDistance:" + String(vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance()));
    #endif

    return vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance() <= kWallDistance;
}


bool Movement::hasWallInFront() {
    #if DEBUG_MOVEMENT
    customPrintln("FrontDistance:" + String(vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance()));
    #endif

    return vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance() <= kHalfTileInMeters;
}

void Movement::maybeResetWithBackWall(const double targetOrientation, const double currentOrientation){
    if (!inResetRoutine_ && counterMovements_ >= kMaxMovements_ && hasWallBehind() ) {
        inResetRoutine_ = true;
        stopMotors();
        pidForward_.setBaseSpeed(kBaseSpeedForwardReset_);
        pidBackward_.setBaseSpeed(kBaseSpeedForwardReset_);
        
        // The encoders reset in moveMotors Backward in the hasTraveledDistanceWithSpeedForBackward function
        moveMotors(MovementState::kBackward, getOrientation(currentOrientation), 0.10, useWallDistance_);

        const double phaseCorrection = getPhaseCorrection(currentOrientation, targetOrientation);
        
        bno_.setPhaseCorrection(phaseCorrection);
        encodersReset_ = true;
        
        #if DEBUG_MOVEMENT
        customPrintln("DistanceToCenter:" + String(distanceToCenter_));
        #endif
        updateDistanceToCenterInTile();

        #if DEBUG_OFFLINE_MOVEMENT
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("DistanceToCenter:" + String(distanceToCenter_));
        udp.endPacket();
        #endif

        // TODO: Clean the parameters of moveMotors
        moveMotors(MovementState::kForward, targetOrientation, distanceToCenter_, useWallDistance_);
        
        pidForward_.setBaseSpeed(kBaseSpeedForward_);
        pidBackward_.setBaseSpeed(kBaseSpeedForward_);
        inResetRoutine_ = false;
        counterMovements_ = 0;
    }

}

double Movement::getPhaseCorrection(const double currentOrientation, const double targetOrientation) {
    const double realOrientation = bno_.getOrientationX();
    double phaseCorrection = realOrientation - targetOrientation;
    #if DEBUG_MOVEMENT
    customPrintln("currentOrientation:" + String(currentOrientation));
    customPrintln("realOrientation:" + String(realOrientation));
    customPrintln("targetOrientation:" + String(targetOrientation));
    #endif

    if (abs(phaseCorrection) < 180) {
        phaseCorrection = phaseCorrection;
    } else if (realOrientation < currentOrientation) {
        phaseCorrection = 360 - abs(phaseCorrection);
    } else {
        phaseCorrection = abs(phaseCorrection) - 360;
    }

    return phaseCorrection;
}

double Movement::getRealTargetDistance(double targetDistance) {
    double initialFrontWallDistance = vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance();
    double initialBackWallDistance = vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance();

    if (initialBackWallDistance >= kUnreachableDistance && initialFrontWallDistance >= kUnreachableDistance) {
       /*  udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("Unreachable");
        udp.endPacket(); */
        return targetDistance;
    }

    if (initialBackWallDistance <= initialFrontWallDistance) {
        initialBackWallDistance = initialBackWallDistance * kMToCm;
        
        const double cm2Center = (uint8_t)(initialBackWallDistance) % kTileLength;

        double distanceToCenter = cm2Center / kMToCm;
        
        if (distanceToCenter <= kIdealDistanceCenter) {
            distanceToCenter = kIdealDistanceCenter - distanceToCenter;
            /* udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("1");
            udp.print("DistanceToCenter:" + String(distanceToCenter));
            udp.endPacket(); */

            return targetDistance + distanceToCenter;
        } else {
            distanceToCenter = distanceToCenter - kIdealDistanceCenter;
            /* udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("2");
            udp.print("DistanceToCenter:" + String(distanceToCenter));
            udp.endPacket(); */
            return targetDistance - distanceToCenter;
        }
    } else {
        initialFrontWallDistance = initialFrontWallDistance * kMToCm;

        double cm2Center = (uint8_t)(initialFrontWallDistance) % kTileLength;

        double distanceToCenter = cm2Center / kMToCm;

        if (distanceToCenter <= kIdealDistanceCenter) {
            distanceToCenter = kIdealDistanceCenter - distanceToCenter;
            /* udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("3");
            udp.print("DistanceToCenter:" + String(distanceToCenter));
            udp.endPacket(); */
            return targetDistance - distanceToCenter;
        } else if (distanceToCenter <= 0.10) {
            distanceToCenter = distanceToCenter - kIdealDistanceCenter;
            /* udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("4");
            udp.print("DistanceToCenter:" + String(distanceToCenter));
            udp.endPacket(); */
            return targetDistance - distanceToCenter;
        } else {
            distanceToCenter = distanceToCenter - kIdealDistanceCenter;
            /* udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("5");
            udp.print("DistanceToCenter:" + String(distanceToCenter));
            udp.endPacket(); */
            return targetDistance - distanceToCenter;
        }
    }
}

double Movement::getDistanceToCenter(double targetDistance) {
    double initialFrontWallDistance = vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance();
    double initialBackWallDistance = vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance();

    if (initialBackWallDistance >= kUnreachableDistance && initialFrontWallDistance >= kUnreachableDistance) {
       /*  udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("Unreachable");
        udp.endPacket(); */
        return targetDistance;
    }

    if (initialBackWallDistance <= initialFrontWallDistance) {
        initialBackWallDistance = initialBackWallDistance * kMToCm;
        
        const double cm2Center = (uint8_t)(initialBackWallDistance) % kTileLength;

        double distanceToCenter = cm2Center / kMToCm;
        
        if (distanceToCenter <= kIdealDistanceCenter) {
            distanceToCenter = kIdealDistanceCenter - distanceToCenter;
            /* udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("1");
            udp.print("DistanceToCenter:" + String(distanceToCenter));
            udp.endPacket(); */
    

            return  distanceToCenter;
        } else {
            distanceToCenter = distanceToCenter - kIdealDistanceCenter;
            // udp.beginPacket(udpServerIP, udpServerPort);
            // udp.print("2");
            // udp.print("DistanceToCenter:" + String(distanceToCenter));
            // udp.endPacket();
        
            return  distanceToCenter;
        }
    } else {
        initialFrontWallDistance = initialFrontWallDistance * kMToCm;

        double cm2Center = (uint8_t)(initialFrontWallDistance) % kTileLength;

        double distanceToCenter = cm2Center / kMToCm;

        if (distanceToCenter <= kIdealDistanceCenter) {
            distanceToCenter = kIdealDistanceCenter - distanceToCenter;
            // udp.beginPacket(udpServerIP, udpServerPort);
            // udp.print("3");
            // udp.print("DistanceToCenter:" + String(distanceToCenter));
            // udp.endPacket();
        
            return  distanceToCenter;
        } else if (distanceToCenter <= 0.10) {
            distanceToCenter = distanceToCenter - kIdealDistanceCenter;
            /* udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("4");
            udp.print("DistanceToCenter:" + String(distanceToCenter));
            udp.endPacket(); */
    
            return  distanceToCenter;
        } else {
            distanceToCenter = distanceToCenter - kIdealDistanceCenter;
            /* udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("5");
            udp.print("DistanceToCenter:" + String(distanceToCenter));
            udp.endPacket(); */
    
            return  distanceToCenter;
        }
    }
}

bool Movement::wasBlackTile() {
    return blackTile_;
}

bool Movement::isBlueTile() {
    return blueTile_;
}

bool Movement::isCheckpointTile() {
    return checkpointTile_;
}

int Movement::directionRamp() {
    const double currentOrientationY = bno_.getOrientationY();
    if (currentOrientationY >= kMinRampOrientation) {
        return 1;
    } else if (currentOrientationY <= -kMinRampOrientation) {
        return -1;
    }
    return 0;
}

void Movement::sendSerialRequest() {
    #if DEBUG_OFFLINE_MOVEMENT
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("Request");
    udp.endPacket();
    #endif
    Serial.println(kSendRequestCode);
    hasReceivedSerial = false;
}

void Movement::checkSerial(double currentOrientation) {
    if (Serial.available() > 0) {
        hasReceivedSerial = true;
        victim = Serial.read();
        screenPrint("Serial Received");
        if (victim != kNoVictimSerialCode) {
            screenPrint("Victim Found");
            saveLastState(getCurrentState(), currentOrientation);
            stopMotors();
            victimFound = true;
            const double rightDistance = vlx[static_cast<uint8_t>(VlxID::kRight)].getRawDistance();
            const double leftDistance = vlx[static_cast<uint8_t>(VlxID::kLeft)].getRawDistance();
            delay(kOneSecInMs);
            if (victim == kHarmedSerialCodeLeft && leftDistance < kWallDistance) {
                screenPrint("Throw 2 medkits left");
                moveServo(servoPosition::kLeft);
                moveServo(servoPosition::kLeft);
            } else if (victim == kHarmedSerialCodeRight && rightDistance < kWallDistance) {
                screenPrint("Throw 2 medkits right");
                moveServo(servoPosition::kRight);
                moveServo(servoPosition::kRight);
            } else if (victim == kStableSerialCodeLeft && leftDistance < kWallDistance) {
                screenPrint("Throw 1 medkit left");
                moveServo(servoPosition::kLeft);
                delay(2000);
            } else if (victim == kStableSerialCodeRight && rightDistance < kWallDistance) {
                screenPrint("Throw 1 medkit right");
                moveServo(servoPosition::kRight);
                delay(2000);
            } else if (victim == kUnharmedSerialCodeLeft || victim == kUnharmedSerialCodeRight){
                screenPrint("Unharmed");
                delay(4000);
            }
            retrieveLastState();
        }
    }
}

char Movement::getVictim() {
    return victim;
}

void Movement::moveServo(servoPosition position) { // Takes two seconds.
    switch (position) {
        case servoPosition::kLeft:
            if (leftStock > 1 && rightStock > 0 || leftStock == 1 && rightStock <= 1) {
                myservo.write(leftAngle);
                --leftStock;
            } else if (leftStock == 1 && rightStock > 1 || leftStock == 0 && rightStock > 0) {
                myservo.write(rightAngle);
                --rightStock;
            }
            delay(kOneSecInMs);
            myservo.write(initialAngle);
            break;
        case servoPosition::kRight:
            if (rightStock > 1 && leftStock > 0 || rightStock == 1 && leftStock <= 1) {
                myservo.write(rightAngle);
                --rightStock;
            } else if (rightStock == 1 && leftStock > 1 || rightStock == 0 && leftStock > 0) {
                myservo.write(leftAngle);
                --leftStock;
            }
            delay(kOneSecInMs);
            myservo.write(initialAngle-6);
            break;
        case servoPosition::kCenter:
            myservo.write(initialAngle);
            break;
    }
    myservo.write(initialAngle);
    delay(250);
    myservo.write(initialAngle+5);
    delay(250);
    myservo.write(initialAngle-5);
    delay(250);
    myservo.write(initialAngle);
    delay(250);
}

bool Movement::getVictimFound() {
    return victimFound;
}

void Movement::screenPrint(const String output){
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.println(output);
    display.display();
    // delay(500);
}

bool Movement::checkForCrashAndCorrect(bool crashLeft, bool crashRight, double currentOrientation, bool useWallDistance) {

    if (crashLeft || crashRight) {
        double frontLeftDistance = vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance();
        double frontRightDistance = vlx[static_cast<uint8_t>(VlxID::kFrontRight)].getRawDistance();
        if (frontLeftDistance <= kMinWallDistance && frontRightDistance <= kMinWallDistance) {
            return false;
        }
    }
    if (crashLeft == true && crashRight == false) {
        currentOrientation = bno_.getOrientationX();
        #if DEBUG_MOVEMENT
        customPrintln("Crash left-");
        #endif
        correctionAfterCrash(true, currentOrientation, useWallDistance);
    }
    
    if (crashRight == true && crashLeft == false) {
        currentOrientation = bno_.getOrientationX();
        #if DEBUG_MOVEMENT
        customPrintln("Crash right-");
        #endif
        correctionAfterCrash(false, currentOrientation, useWallDistance);
    }

    return true;
}

void Movement::printEncoderTics() {
    for (uint8_t i = 0; i < kNumberOfWheels; ++i) {
        // customPrintln("Motor" + String(i) + "Tics:" + String(motor[i].getTics()));
    }
}

void Movement::resetSerial() {
    // while (Serial.available() <= 0) { // Wait for the serial that was left unread.
    //     screenPrint("Waiting for Serial");
    // }
    screenPrint("Serial Reseted");
    if (Serial.available() > 0) {
        Serial.read(); // Read the serial that was left unread.
    }
    #if DEBUG_OFFLINE_MOVEMENT
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("Reset");
    udp.endPacket();
    #endif
    Serial.println(kResetSerialCode); // Send serial to reset the count of detections in Jetson.
    delay(500);
    sendSerialRequest();
}

void Movement::calibrateColors() {
    screenPrint("checking blue");
    delay(kOneSecInMs);
    while (digitalRead(Pins::buttonPin) == HIGH) {}
    tcs_.getBlueRanges();
    screenPrint("checking black");
    delay(kOneSecInMs);
    while (digitalRead(Pins::buttonPin) == HIGH) {}
    tcs_.getBlackRanges();
    screenPrint("checking checkpoint");
    delay(kOneSecInMs);
    while (digitalRead(Pins::buttonPin) == HIGH) {}
    tcs_.getCheckpointRanges();
    screenPrint("finished checking");
    delay(kOneSecInMs);
}

void Movement::checkForLackOfProgress() {
    if (digitalRead(Pins::buttonPin) == LOW) {
        lackOfProgress_ = true;
    }
}

bool Movement::getLackOfProgress() {
    return lackOfProgress_;
}

void Movement::resetLackOfProgress() {
    lackOfProgress_ = false;
    // currentOrientation_ = 0;
}

bool Movement::onFlatGround() {
    // customPrintln(String("OrientationY:") + String(bno_.getOrientationY()));
    return bno_.getOrientationY() < kHorizontalAngleError && bno_.getOrientationY() > -kHorizontalAngleError;
}

void Movement::printColorRanges() {
    // tcs_.getRanges();
}

void Movement::maybeGoBackwards(const double currentOrientation) {
    const unsigned long timeDiff = millis() - timePrevTurn_;

    if (timeDiff < timeToTurn_) {
        return;
    }
    stopMotors();
    kITurn = 0.00005;
    moveMotors(MovementState::kBackward, getOrientation(currentOrientation), 0.02, false);
    timePrevTurn_ = millis();
}

void Movement::weightPID(const double targetOrientation, const double currentOrientation, const double targetDistance, double currentDistance, double& speedLeft, double& speedRight) {
    pidForward_.computeStraight(targetOrientation, currentOrientation, speedLeft, speedRight);
    const double speedLeftBno = speedLeft;
    const double speedRightBno = speedRight;
    
    pidWallAlignment_.computeDistance(targetDistance, currentDistance, speedLeft, speedRight);
    const double speedLeftVlx = speedLeft;
    const double speedRightVlx = speedRight;

    const double weightedSpeedLeft = speedLeftBno * kWeightBNO + speedLeftVlx * kWeightVLX;
    const double weightedSpeedRight = speedRightBno * kWeightBNO + speedRightVlx * kWeightVLX;
}