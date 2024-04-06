
#include "Movement.h"
#include "Pins.h"
#include "CustomSerial.h"
#include "VLX.h"

#define DEBUG_MOVEMENT 0
#define DEBUG_OFFLINE_MOVEMENT 0

/* #include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "RoBorregos2";
const char* password = "RoBorregos2024";
const char* udpServerIP = "192.168.0.109"; // Replace with your Python script's IP address
const int udpServerPort = 1238;

WiFiUDP udp;  */
#if DEBUG_OFFLINE_MOVEMENT
#endif



Movement::Movement() {
    
}

void Movement::setup() {
    this->prevTimeTraveled_ = millis();
    // WARNING? : The pidDummy_ doesn't work apparently the first pid won't work so by first writing an unused pid it will work with all the pids
    this->pidDummy_.setTunnings(kPForward, kIForward, kDForward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidForward_.setTunnings(kPForward, kIForward, kDForward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidBackward_.setTunnings(kPBackward, kIBackward, kDBackward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidTurn_.setTunnings(kPTurn, kITurn, kDTurn, kTurnMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedTurn_, kMaxOrientationError);

    /* WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(kOneSecInMs);
        customPrintln("Connecting to WiFi...");
    }
    customPrintln("Connected to WiFi");

    udp.begin(udpServerPort);
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("Connected to WiFi");
    udp.endPacket(); */
    #if DEBUG_OFFLINE_MOVEMENT
    #endif

    setupInternal(MotorID::kFrontLeft);
    setupInternal(MotorID::kFrontRight);
    setupInternal(MotorID::kBackLeft);
    setupInternal(MotorID::kBackRight);
    bno_.setupBNO();
    Wire.begin();
    // TODO: Make a loop in which it setup the VLXs
    setupVlx(VlxID::kFrontRight);
    setupVlx(VlxID::kBack);
    setupVlx(VlxID::kLeft);
    setupVlx(VlxID::kRight);
    setupVlx(VlxID::kFrontLeft);

    setupLimitSwitch(LimitSwitchID::kLeft);
    setupLimitSwitch(LimitSwitchID::kRight);

    setupTCS();
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
void Movement::moveMotorsInADirection(double targetOrientation, bool moveForward){
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
        pidForward_.computeStraight(targetOrientation, currentOrientation, speedLeft, speedRight);
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

void Movement::getAllWallsDistances(double wallDistances[kNumberOfVlx]) {
    for (const auto& vlxDirection : vlxDirections) {
        const uint8_t index = static_cast<uint8_t>(vlxDirection);
        wallDistances[index] = getWallDistance(vlxDirection);
    }

    #if DEBUG_MOVEMENT
    customPrintln("FrontRight:" + String(wallDistances[static_cast<uint8_t>(VlxID::kFrontRight)]));
    customPrintln("Back:" + String(wallDistances[static_cast<uint8_t>(VlxID::kBack)]));
    customPrintln("Left:" + String(wallDistances[static_cast<uint8_t>(VlxID::kLeft)]));
    customPrintln("Right:" + String(wallDistances[static_cast<uint8_t>(VlxID::kRight)]));
    customPrintln("FrontLeft:" + String(wallDistances[static_cast<uint8_t>(VlxID::kFrontLeft)]));
    #endif
}

uint8_t Movement::checkWallsDistances() {
    // 00000
    // FBLRF
    uint8_t wallDetected = 0;
    for (uint8_t i = 0; i < kNumberOfVlx; ++i) {
        wallDetected |= (wallDistances[i] < kWallDistance? 1:0) << i;
    }

    return wallDetected;
}

double Movement::getWallDistance(const VlxID vlxId) {
    return wallDistances[static_cast<uint8_t>(vlxId)];
}

// TODO: Change the targetOrientation to a reference variable
void Movement::goForward() {
    moveMotors(MovementState::kForward, 0, 0.3);
}

void Movement::goBackward() {
    moveMotors(MovementState::kBackward, 0, 0.3);
}

void Movement::turnLeft() {
    moveMotors(MovementState::kTurnLeft, 270, 0);
}

void Movement::turnRight() {
    moveMotors(MovementState::kTurnRight, 90, 0);
}

void Movement::rampMovement() {
    moveMotors(MovementState::kRamp, 0, 0);
}

void Movement::moveMotors(const MovementState state, const double targetOrientation, double targetDistance, bool useWallDistance, bool needReset) {
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

    const double initialFrontWallDistance = vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance();
    const double initialBackWallDistance = vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance();
    bool moveForward = false;
    ++counterMovements_;
    
    customPrintln("CounterMovements:" + String(counterMovements_));
    switch (state)
    {
        case (MovementState::kStop): {
            stopMotors();
            break;
        }
        case (MovementState::kForward): {
            moveForward = true;
            currentState_ = MovementState::kForward;
            blackTile_ = false;
            blueTile_ = false;
            checkpointTile_ = false;
            finishedMovement_ = false;

            double frontWallDistance = vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance();
            double backWallDistance = vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance();
            
            // TODO: improve the way to give a targetDistance with vlx
            // targetDistance = getRealTargetDistance(targetDistance);

            /* udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("targetDistance:" + String(targetDistance));
            udp.endPacket(); */
            #if DEBUG_OFFLINE_MOVEMENT
            #endif

            while (hasTraveledDistanceWithSpeed(targetDistance) == false) {
                frontWallDistance = vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance();
                /* udp.beginPacket(udpServerIP, udpServerPort);
                udp.print("vlxFrontLeft:" + String(frontWallDistance));
                udp.endPacket(); */
                if (frontWallDistance <= kMinWallDistance) {
                    stopMotors();
                    break;
                }

                // checkColors();

                // customPrintln("Color:" + String(getTCSInfo()));
                // customPrintln("blackTile" + String(blackTile_));
                #if DEBUG_MOVEMENT
                #endif
                // if (wasBlackTile()) {
                //     customPrintln("Black tile detected");
                //     return;
                // }

                crashLeft = limitSwitch_[leftLimitSwitch].getState();
                crashRight = limitSwitch_[rightLimitSwitch].getState();

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
                customPrintln("Rampa: " + String(isRamp()));
                moveMotorsInADirection(targetOrientation, true);

                // TODO: Make its own function named checkForCrashAndCorrect()
                if (crashLeft == true && crashRight == false) {
                    #if DEBUG_MOVEMENT
                    customPrintln("Crash left-");
                    #endif
                    correctionAfterCrash(true, currentOrientation, useWallDistance);
                }
                
                if (crashRight == true && crashLeft == false) {
                    #if DEBUG_MOVEMENT
                    customPrintln("Crash right-");
                    customPrintln("Encodersssss");
                    #endif
                    correctionAfterCrash(false, currentOrientation, useWallDistance);
                }
            }
            stopMotors();

            frontWallDistance = vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance();
            backWallDistance = vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance();

            if (inResetRoutine_ == false) {
                while (abs(weightMovementVLX(backWallDistance, frontWallDistance, 
                                            initialBackWallDistance, initialFrontWallDistance, 
                                            moveForward, targetDistance) - targetDistance) >= 1 ) {
                    
                    customPrintln("vlxFrontLeft:" + String(frontWallDistance));
                    customPrintln("vlxBack:" + String(backWallDistance));
                
                    if (!moveForward || backWallDistance < kUnreachableDistance) {
                        backWallDistance = vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance();
                    }
                    if (moveForward || frontWallDistance < kUnreachableDistance) {
                        frontWallDistance = vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance();   
                    }

                    if (moveForward && frontWallDistance <= kMinWallDistance) {
                        stopMotors();
                        break;
                    }

                    // udp.beginPacket(udpServerIP, udpServerPort);
                    // udp.print("vlxFrontLeft:" + String(frontWallDistance));
                    // udp.print(" ");
                    // udp.print("vlxBack:" + String(backWallDistance));
                    // udp.print(" ");
                    // udp.print("targetDistance:" + String(targetDistance));
                    // udp.print(" ");
                    // udp.print("weightMovement:" + String(weightMovementVLX(backWallDistance, frontWallDistance, 
                    //                          initialBackWallDistance, initialFrontWallDistance, 
                    //                          moveForward, targetDistance)));
                    // udp.endPacket();
                    
                    // checkColors();
                    
                    // customPrintln("Color:" + String(getTCSInfo()));
                    // customPrintln("blackTile" + String(blackTile_));
                    #if DEBUG_MOVEMENT
                    #endif
                    /* if (wasBlackTile()) {
                        customPrintln("Black tile detected");
                        return;
                    } */
                    
                    if (moveForward){
                        crashLeft = limitSwitch_[leftLimitSwitch].getState();
                        crashRight = limitSwitch_[rightLimitSwitch].getState();
                    }

                    if (crashLeft == true && crashRight == false) {
                        #if DEBUG_MOVEMENT
                        customPrintln("Crash left-");
                        #endif
                        correctionAfterCrash(true, currentOrientation, useWallDistance);
                    }
                    
                    if (crashRight == true && crashLeft == false) {
                        #if DEBUG_MOVEMENT
                        customPrintln("Crash right-");
                        customPrintln("Encodersssss");
                        #endif
                        correctionAfterCrash(false, currentOrientation, useWallDistance);
                    }

                    #if DEBUG_OFFLINE_MOVEMENT
                    udp.beginPacket(udpServerIP, udpServerPort);
                    udp.print("targetOrientation" + String(targetOrientation));
                    udp.print(" ");
                    udp.print("CurrentOreintation" + String(currentOrientation));
                    udp.print(" ");
                    udp.print("CurrentDistance" + String(targetDistance));
                    udp.endPacket();
                    #endif

                    //customPrintln("moveForward:" + String(moveForward));
                    moveMotorsInADirection(targetOrientation, moveForward);
                }
            }

            finishedMovement_ = true;
            // checkColors();

            #if DEBUG_OFFLINE_MOVEMENT
            udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("allDistanceTraveled___:" + String(allDistanceTraveled_));
            udp.endPacket();
            #endif
            allDistanceTraveled_ = 0;

            stopMotors();
            
            break;
        }
        case (MovementState::kBackward): {
            moveForward = false;
            if (inResetRoutine_) {
                resetDistanceTraveled_ = true;
            }

            while (hasTraveledDistanceWithSpeed(targetDistance) == false) {
                #if DEBUG_OFFLINE_MOVEMENT
                udp.beginPacket(udpServerIP, udpServerPort);
                udp.print("--------------------------------------------------------------");
                udp.endPacket();
                #endif
                moveMotorsInADirection(targetOrientation, moveForward);
            }

            const double desiredWallDistance = initialFrontWallDistance  + targetDistance;

            // TODO: change the way to check the wall distance
            while (useWallDistance == true && hasTraveledWallDistance(desiredWallDistance, getWallDistance(VlxID::kFrontRight)) == false) {
                moveMotorsInADirection(targetOrientation, moveForward);
            }

            stopMotors();
            
            break;
        }
        // TODO: change MotorStarte of turnRigth and left to make an oneself motorState and with that I mean turn 
        case (MovementState::kTurnLeft): {
            turnMotors(targetOrientation, targetDistance, currentOrientation);

            break;
        }
        case (MovementState::kTurnRight): {
            turnMotors(targetOrientation, targetDistance, currentOrientation);

            break;
        }
        case (MovementState::kRamp): {
            #if DEBUG_MOVEMENT
            customPrintln("kRamp");
            #endif
            rampDetected = isRamp();
            while (rampDetected) {
                moveMotorsInADirection(getOrientation(currentOrientation), true);
                rampDetected = isRamp();
            }
            
            const unsigned long timePrevRamp = millis();
            unsigned long timeDiff = millis() - timePrevRamp;
            customPrintln("timePrevRamp:" + String(timePrevRamp));
            while (timeDiff < kTimeAfterRamp) {
                customPrintln("TimeDiff:" + String(timeDiff));
                timeDiff = millis() - timePrevRamp;
                moveMotorsInADirection(getOrientation(currentOrientation), true);
            }
            
            stopMotors();

            customPrintln("Ramp finished");
            customPrintln("hasWallInFront:" + String(hasWallInFront()));
            customPrintln("vlxFrontLeft: " + String(vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance()));
            if (hasWallInFront()) {
                customPrintln("Wall in front");
                moveMotors(MovementState::kForward, targetOrientation, 0.30);
            }

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
        moveMotors(MovementState::kBackward, getOrientation(currentOrientation - crashDeltaOrientation_), crashDeltaDistance_ / 2, useWallDistance);
        moveMotors(MovementState::kTurnRight, getOrientation(currentOrientation - crashDeltaOrientation_), 0);
        moveMotors(MovementState::kBackward, getOrientation(currentOrientation + crashDeltaOrientation_), crashDeltaDistance_ / 2, useWallDistance);
        moveMotors(MovementState::kTurnLeft, getOrientation(currentOrientation + crashDeltaOrientation_), 0);
    } else {
        #if DEBUG_MOVEMENT
        customPrintln("Crash left--------");
        #endif
        moveMotors(MovementState::kBackward, getOrientation(currentOrientation - crashDeltaOrientation_), crashDeltaDistance_ / 2, useWallDistance);
        moveMotors(MovementState::kTurnLeft, getOrientation(currentOrientation + crashDeltaOrientation_), 0);
        moveMotors(MovementState::kBackward, getOrientation(currentOrientation + crashDeltaOrientation_), crashDeltaDistance_ / 2, useWallDistance);
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

    while (abs(pidTurn_.computeErrorOrientation(targetOrientation, currentOrientation)) > kMaxOrientationError) {

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
    if (timeTraveled_ < kSampleTimeTraveled) {
        return false;
    }
    // TODO: Improve the way to calculate the average speed
    const double averageSpeed = (motor[static_cast<uint8_t>(MotorID::kBackLeft)].getSpeed() +
                                 motor[static_cast<uint8_t>(MotorID::kBackRight)].getSpeed() +
                                 motor[static_cast<uint8_t>(MotorID::kFrontLeft)].getSpeed() +
                                 motor[static_cast<uint8_t>(MotorID::kFrontRight)].getSpeed()) / kNumberOfWheels;
    
    const double distanceTraveled = averageSpeed * (timeTraveled_ / (double)kOneSecInMs);

    allDistanceTraveled_ += distanceTraveled;
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
    /* udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("FALSE");
    udp.endPacket(); */
    return false;
}

bool Movement::hasTraveledWallDistance(double targetDistance, double currentDistance) {
    const double distanceDiff = (targetDistance - currentDistance);
    return abs(distanceDiff) < kMaxDistanceError;
}

void Movement::printTCS() {
    tcs_.printRGB();
    tcs_.getColor();
}

char Movement::getTCSInfo() {
    printTCS();
    return tcs_.getColorWithThresholds();
} 

void Movement::rgbTCSClear() {
    tcs_.printRGBC();
}

char Movement::checkColors() {
    const char color = getTCSInfo();
    // customPrintln("Color:" + String(color));
    if (color == kBlackColor) {
        blackTile_ = true;
        const double desiredDistance = allDistanceTraveled_;
        targetDistance_ = desiredDistance;
        allDistanceTraveled_ = 0;
        customPrintln("blackTile__" + String(blackTile_));
        stopMotors();
        moveMotors(MovementState::kBackward, targetOrientation_, targetDistance_);
        return color;
    } else if (color == kBlueColor && finishedMovement_ == true) {
        blueTile_ = true;
        stopMotors();
        customPrintln("DETECTED BLUE TILE");
        delay(kFiveSeconds_);
        
        return color;
    } else if (color == kRedColor) {
        checkpointTile_ = true;
        stopMotors();
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
    return (weightMovement);
    
}

double Movement::weightMovementVLX(const double currentDistanceBack, const double currentDistanceFront, const double initialVlxDistanceBack, const double initialVlxDistanceFront, bool& moveForward, double targetDistance) {
    double vlxDistanceTraveled = 0;
    bool rampDetected = isRamp();

    if (rampDetected) {
        #if DEBUG_MOVEMENT
        customPrintln("Ramp detected");
        #endif
        return targetDistance;
    }

    if (currentDistanceBack < currentDistanceFront) {
        if (initialVlxDistanceBack >= kUnreachableDistance || currentDistanceBack >= kUnreachableDistance) {
            /* udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("Unreachable");
            udp.print(" ");
            udp.print("initialVlxDistanceBack:" + String(initialVlxDistanceBack));
            udp.endPacket(); */
            #if DEBUG_OFFLINE_MOVEMENT
            #endif
            return targetDistance;
        }
        vlxDistanceTraveled = currentDistanceBack - initialVlxDistanceBack;
        /* udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("Back");
        udp.print(" ");
        udp.print("currentDistanceBack:" + String(currentDistanceBack));
        udp.print(" ");
        udp.print("currentDistanceFront:" + String(currentDistanceFront));
        udp.print(" ");
        udp.print("initialVlxDistanceBack:" + String(initialVlxDistanceBack));
        udp.print(" ");
        udp.print("vlxDistanceTraveled:" + String(vlxDistanceTraveled));
        udp.endPacket(); */
        #if DEBUG_OFFLINE_MOVEMENT
        #endif
    } else {
        if (initialVlxDistanceFront >= kUnreachableDistance || currentDistanceFront >= kUnreachableDistance) {
            /*udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("Unreachable");
            udp.print(" ");
            udp.print("initialVlxDistanceFront:" + String(initialVlxDistanceFront));
            udp.endPacket(); */
            #if DEBUG_OFFLINE_MOVEMENT
            #endif
            return targetDistance;
        }
        vlxDistanceTraveled = initialVlxDistanceFront - currentDistanceFront;
        /* udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("Front");
        udp.print(" ");
        udp.print("currentDistanceBack:" + String(currentDistanceBack));
        udp.print(" ");
        udp.print("currentDistanceFront:" + String(currentDistanceFront));
        udp.print(" ");
        udp.print("initialVlxDistanceFront:" + String(initialVlxDistanceFront));
        udp.print(" ");
        udp.print("vlxDistanceTraveled:" + String(vlxDistanceTraveled));
        udp.endPacket(); */
        #if DEBUG_OFFLINE_MOVEMENT
        #endif
    }
    /* udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("allDistanceTraveled:" + String(allDistanceTraveled_));
    udp.print(" ");
    udp.print("vlxDistanceTraveled:" + String(vlxDistanceTraveled));
    udp.print(" ");
    udp.print("allMovement:" + String(allDistanceTraveled_ * kWeightEncoders + vlxDistanceTraveled * kWeightVlx));
    udp.endPacket(); */
    #if DEBUG_OFFLINE_MOVEMENT
    #endif
    const double weightMovement = vlxDistanceTraveled * kWeightVlx;
    customPrintln("vlxDistanceTraveled:" + String(vlxDistanceTraveled) + " " + "targetDistance:" + String(targetDistance));
    moveForward = (int8_t)((vlxDistanceTraveled - targetDistance) * kMToCm) < 0;
    customPrintln("moveForward:" + String(moveForward));
    customPrintln("calculation:" + String((int8_t)((vlxDistanceTraveled - targetDistance) * kMToCm)));
    /* udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("moveForward:" + String(moveForward));
    udp.endPacket(); */
    return weightMovement;

}

// double Movement::hasTraveledDistanceWithVlx(const double currentDistanceBack, const double currentDistanceFront, bool& moveForward, double targetDistance) {
//     double vlxDistanceTraveled = 0;
//     bool rampDetected = isRamp();

//     if (rampDetected) {
//         #if DEBUG_MOVEMENT
//         customPrintln("Ramp detected");
//         #endif
//         return targetDistance;
//     }

//     if (currentDistanceBack < currentDistanceFront) {
//         vlxDistanceTraveled = currentDistanceBack - initialVlxDistanceBack;
//         /* udp.beginPacket(udpServerIP, udpServerPort);
//         udp.print("Back");
//         udp.print(" ");
//         udp.print("currentDistanceBack:" + String(currentDistanceBack));
//         udp.print(" ");
//         udp.print("currentDistanceFront:" + String(currentDistanceFront));
//         udp.print(" ");
//         udp.print("initialVlxDistanceBack:" + String(initialVlxDistanceBack));
//         udp.print(" ");
//         udp.print("vlxDistanceTraveled:" + String(vlxDistanceTraveled));
//         udp.endPacket(); */
//         #if DEBUG_OFFLINE_MOVEMENT
//         #endif
//     } else {
//         if (initialVlxDistanceFront >= kUnreachableDistance || currentDistanceFront >= kUnreachableDistance) {
//             /*udp.beginPacket(udpServerIP, udpServerPort);
//             udp.print("Unreachable");
//             udp.print(" ");
//             udp.print("initialVlxDistanceFront:" + String(initialVlxDistanceFront));
//             udp.endPacket(); */
//             #if DEBUG_OFFLINE_MOVEMENT
//             #endif
//             return targetDistance;
//         }
//         vlxDistanceTraveled = initialVlxDistanceFront - currentDistanceFront;
//         /* udp.beginPacket(udpServerIP, udpServerPort);
//         udp.print("Front");
//         udp.print(" ");
//         udp.print("currentDistanceBack:" + String(currentDistanceBack));
//         udp.print(" ");
//         udp.print("currentDistanceFront:" + String(currentDistanceFront));
//         udp.print(" ");
//         udp.print("initialVlxDistanceFront:" + String(initialVlxDistanceFront));
//         udp.print(" ");
//         udp.print("vlxDistanceTraveled:" + String(vlxDistanceTraveled));
//         udp.endPacket(); */
//         #if DEBUG_OFFLINE_MOVEMENT
//         #endif
//     }
//     /* udp.beginPacket(udpServerIP, udpServerPort);
//     udp.print("allDistanceTraveled:" + String(allDistanceTraveled_));
//     udp.print(" ");
//     udp.print("vlxDistanceTraveled:" + String(vlxDistanceTraveled));
//     udp.print(" ");
//     udp.print("allMovement:" + String(allDistanceTraveled_ * kWeightEncoders + vlxDistanceTraveled * kWeightVlx));
//     udp.endPacket(); */
//     #if DEBUG_OFFLINE_MOVEMENT
//     #endif
//     const double weightMovement = vlxDistanceTraveled * kWeightVlx;
//     customPrintln("vlxDistanceTraveled:" + String(vlxDistanceTraveled) + " " + "targetDistance:" + String(targetDistance));
//     moveForward = (vlxDistanceTraveled - targetDistance) > 0;
//     /* udp.beginPacket(udpServerIP, udpServerPort);
//     udp.print("moveForward:" + String(moveForward));
//     udp.endPacket(); */
//     return weightMovement;
// }

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

    return vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance() <= 0.22;
}

void Movement::maybeResetWithBackWall(const double targetOrientation, const double currentOrientation){
    if (!inResetRoutine_ && counterMovements_ >= kMaxMovements_ && hasWallBehind() ) {

        inResetRoutine_ = true;
        stopMotors();
        pidForward_.setBaseSpeed(kBaseSpeedForwardReset_);
        pidBackward_.setBaseSpeed(kBaseSpeedForwardReset_);
        
        // The encoders reset in moveMotors Backward in the hasTraveledDistanceWithSpeedForBackward function
        moveMotors(MovementState::kBackward, getOrientation(currentOrientation), kHalfTileInMeters, useWallDistance_, true);

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

        moveMotors(MovementState::kForward, targetOrientation, distanceToCenter_, useWallDistance_, true);
        
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
            /* udp.beginPacket(udpServerIP, udpServerPort);
            udp.print("2");
            udp.print("DistanceToCenter:" + String(distanceToCenter));
            udp.endPacket(); */
            return  distanceToCenter;
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
            return  - distanceToCenter;
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