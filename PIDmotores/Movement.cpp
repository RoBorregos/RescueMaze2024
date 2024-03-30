
#include "Movement.h"
#include "Pins.h"
#include "CustomSerial.h"
#include "VLX.h"

#define DEBUG_MOVEMENT 0
#define DEBUG_OFFLINE_MOVEMENT 0

#if DEBUG_OFFLINE_MOVEMENT
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "RoBorregos2";
const char* password = "RoBorregos2024";
const char* udpServerIP = "192.168.0.107"; // Replace with your Python script's IP address
const int udpServerPort = 1235;

WiFiUDP udp;
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

    #if DEBUG_OFFLINE_MOVEMENT
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(kOneSecInMs);
        customPrintln("Connecting to WiFi...");
    }
    customPrintln("Connected to WiFi");

    udp.begin(udpServerPort);
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
        wallDetected |= (wallDistances[i] < kMinWallDistance? 1:0) << i;
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

void Movement::moveMotors(const MovementState state, const double targetOrientation, const double targetDistance, bool useWallDistance) {
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

            double frontWallDistance = vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance();
            double backWallDistance = vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance();

            while (weightMovement(backWallDistance, frontWallDistance, initialBackWallDistance, initialFrontWallDistance) <= targetDistance) {

                if (frontWallDistance <= kMinFrontWallDistance) {
                    break;
                }

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
                
                moveMotorsInADirection(targetOrientation, moveForward);

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

                #if DEBUG_MOVEMENT
                customPrintln("targetDistance:" + String(targetDistance));
                #endif

                frontWallDistance = vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance();
                backWallDistance = vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance();

                // TODO: improve this condition
                if (hasTraveledDistanceWithSpeed(targetDistance) == true) {
                    break;
                } 
            }

            stopMotors();
            
            break;
        }
        case (MovementState::kBackward): {
            moveForward = false;
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

            stopMotors();
            const unsigned long timePrevRamp = millis();
            unsigned long timeDiff = millis() - timePrevRamp;
            while (timeDiff < kTimeAfterRamp) {
                timeDiff = millis() - timePrevRamp;
                moveMotorsInADirection(getOrientation(currentOrientation), true);
            }

            stopMotors();
            break;
        }
    }

    maybeResetWithBackWall(targetOrientation, currentOrientation, moveForward);
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

    if (allDistanceTraveled_ >= distance) {

        allDistanceTraveled_ = 0;

        #if DEBUG_OFFLINE_MOVEMENT
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("totalDistanceTraveled:" + String(allDistanceTraveled_));
        udp.endPacket();
        #endif
        return true;
    }
    
    return false;
}

bool Movement::hasTraveledWallDistance(double targetDistance, double currentDistance) {
    const double distanceDiff = (targetDistance - currentDistance);
    return abs(distanceDiff) < kMaxDistanceError;
}

bool Movement::isRamp() {
    
    const double currentOrientationY = bno_.getOrientationY();
    #if DEBUG_MOVEMENT
    customPrintln("OrientationY:" + String(currentOrientationY));
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

double Movement::weightMovement(double currentDistanceBack, double currentDistanceFront, double initialVlxDistanceBack, double initialVlxDistanceFront) {
    double vlxDistanceTraveled = 0;
    bool rampDetected = isRamp();

    if (rampDetected) {
        #if DEBUG_MOVEMENT
        customPrintln("Ramp detected");
        #endif
        return allDistanceTraveled_;
    }

    if (currentDistanceBack > kUnreachableDistance && currentDistanceFront > kUnreachableDistance) {
        #if DEBUG_OFFLINE_MOVEMENT
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("Unreachable");
        udp.print(" ");
        udp.print("currentDistanceBack:" + String(currentDistanceBack));
        udp.print(" ");
        udp.print("currentDistanceFront:" + String(currentDistanceFront));
        udp.print(" ");
        udp.print("allDistanceTraveled:" + String(allDistanceTraveled_));
        udp.endPacket();
        #endif
        return allDistanceTraveled_;
    }

    if (currentDistanceBack < currentDistanceFront) {
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
    udp.print("Distance: ");
    udp.print(weightMovemnt(vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance(), vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance(), initialBackWallDistance, initialFrontWallDistance));
    udp.endPacket();
    #endif

    return (allDistanceTraveled_ * kWeightEncoders + vlxDistanceTraveled * kWeightVlx);
    
}

void Movement::updateDistanceToCenterInTile() {
    distanceToCenter_ = (kTileLength - kLargeOfRobot) / 2;
    distanceToCenter_ = distanceToCenter_ / kMToCm;

    #if DEBUG_OFFLINE_MOVEMENT
    udp.beginPacket(udpServerIP, udpServerPort);
    udp.print("kTileLength:" + String(kTileLength));
    udp.print(" ");
    udp.print("kLargeOfRobot:" + String(kLargeOfRobot));
    udp.print(" ");
    udp.print("distanceToCenter__:" + String(distanceToCenter_));
    udp.endPacket();
    #endif
}

bool Movement::hasWallBehind() {
    #if DEBUG_MOVEMENT
    customPrintln("BackDistance:" + String(vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance()));
    #endif

    return vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance() <= kMaxWallDistance_;
}

void Movement::maybeResetWithBackWall(const double targetOrientation, double currentOrientation, bool moveForward){
    if (!inResetRoutine_ && counterMovements_ >= kMaxMovements_ && hasWallBehind() ) {
        inResetRoutine_ = true;
        stopMotors();
        pidForward_.setBaseSpeed(kBaseSpeedForwardReset_);
        pidBackward_.setBaseSpeed(kBaseSpeedForwardReset_);

        // The encoders reset in moveMotors Backward in the hasTraveledDistanceWithSpeedForBackward function
        moveMotors(MovementState::kBackward, getOrientation(currentOrientation), kHalfTileInMeters, false);

        const double phaseCorrection = getPhaseCorrection(currentOrientation, targetOrientation);
        
        bno_.setPhaseCorrection(phaseCorrection);
        encodersReset_ = true;
        
        moveForward = true;
        #if DEBUG_MOVEMENT
        customPrintln("DistanceToCenter:" + String(distanceToCenter_));
        #endif
        updateDistanceToCenterInTile();


        #if DEBUG_OFFLINE_MOVEMENT
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("DistanceToCenter:" + String(distanceToCenter_));
        udp.endPacket();
        #endif

        moveMotors(MovementState::kForward, targetOrientation, distanceToCenter_, moveForward);
        
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
        #if DEBUG_OFFLINE_MOVEMENT
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("1");
        udp.endPacket();
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("PhaseCorrection: " + String(phaseCorrection));
        udp.endPacket();
        #endif
    } else if (realOrientation < currentOrientation) {
        phaseCorrection = 360 - abs(phaseCorrection);
        #if DEBUG_OFFLINE_MOVEMENT
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("2");
        udp.endPacket();
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("PhaseCorrection: " + String(phaseCorrection));
        udp.endPacket();
        #endif
    } else {
        phaseCorrection = abs(phaseCorrection) - 360;
        #if DEBUG_OFFLINE_MOVEMENT
        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("3");
        udp.endPacket();

        udp.beginPacket(udpServerIP, udpServerPort);
        udp.print("PhaseCorrection: " + String(phaseCorrection));
        udp.endPacket();
        #endif
    }

    return phaseCorrection;
}