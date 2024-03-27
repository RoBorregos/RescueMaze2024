
#include "Movement.h"
#include "Pins.h"
#include "CustomSerial.h"
#include "VLX.h"

#define DEBUG_MOVEMENT 0

Movement::Movement() {
    
}

void Movement::setup() {
    this->prevTimeTraveled_ = millis();
    // WARNING? : The pidDummy_ doesn't work apparently the first pid won't work so by first writing an unused pid it will work with all the pids
    this->pidDummy_.setTunnings(kPForward, kIForward, kDForward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidForward_.setTunnings(kPForward, kIForward, kDForward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidBackward_.setTunnings(kPBackward, kIBackward, kDBackward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidTurn_.setTunnings(kPTurn, kITurn, kDTurn, kTurnMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedTurn_, kMaxOrientationError);
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

double Movement::getDistanceToCenter() {
    double distanceBack = 0;
    double cmToCenterFront = 0;
    distanceBack = vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance();
    distanceBack *= kMToCm;
    cmToCenterFront = ((uint8_t)distanceBack % kTileLength) * 30;

    return cmToCenterFront;
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

            /* bool flag = false;
            if (initialFrontWallDistance < 0.45){
                flag = true;
                while (useWallDistance == true && hasTraveledWallDistance(kMaxDistanceError, vlx[0].getRawDistance()) == false) {
                    // crashLeft = limitSwitch_[leftLimitSwitch].getState();
                    // crashRight = limitSwitch_[rightLimitSwitch].getState();
                    // customPrintln("InitialFrontWallDistance:" + String(initialFrontWallDistance));
                    // customPrintln("targetDistance:" + String(targetDistance));
                    // customPrintln(vlx[0].getRawDistance());
                    // customPrintln(vlx[1].getRawDistance());
                    // customPrintln(vlx[2].getRawDistance());
                    // customPrintln(vlx[3].getRawDistance());
                    moveMotorsInADirection(targetOrientation, moveForward);
                    //customPrintln("DesiredWallDistance:" + String(0.06)); 
                    if (counterMovements_ >= 4 && vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance() < 0.10) {
                        flag = false;
                        break;
                    }

                    if (crashLeft == true && crashRight == false) {
                        correctionAfterCrash(true, currentOrientation, useWallDistance);
                    }

                    if (crashRight == true && crashLeft == false) {
                        correctionAfterCrash(false, currentOrientation, useWallDistance);
                    }

                }
            } */
            /* while ((vlx[0].getRawDistance() <= 0.06 && hasTraveledDistanceWithSpeed(targetDistance) == false) == false){
                crashLeft = limitSwitch_[leftLimitSwitch].getState();
                crashRight = limitSwitch_[rightLimitSwitch].getState(); 
                rampDetected = isRamp();
                
                moveMotorsInADirection(targetOrientation, moveForward);
                
                if (rampDetected) {
                    #if DEBUG_MOVEMENT
                    customPrintln("Ramp detected");
                    #endif
                    useWallDistance = false;
                }

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
    
                //checkWallsDistances();
            } */
            
            // const double desiredWallDistance = initialFrontWallDistance - targetDistance;

            while (weightMovemnt(vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance(), vlx[static_cast<uint8_t>(VlxID::kFrontLeft)].getRawDistance(), initialBackWallDistance, initialFrontWallDistance) <= targetDistance) {

                // crashLeft = limitSwitch_[leftLimitSwitch].getState();
                // crashRight = limitSwitch_[rightLimitSwitch].getState();
                rampDetected = isRamp();

                
                moveMotorsInADirection(targetOrientation, moveForward);
                
                if (rampDetected) {
                    #if DEBUG_MOVEMENT
                    customPrintln("Ramp detected");
                    #endif
                    useWallDistance = false;
                }

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

                hasTraveledDistanceWithSpeed(targetDistance);
            }
            allDistanceTraveled_ = 0;

            // TODO: Change the way to check the wall distance  

            stopMotors();
            
            break;
        }
        case (MovementState::kBackward): {
            moveForward = false;
            while (hasTraveledDistanceWithSpeedForBackward(targetDistance) == false) {
                moveMotorsInADirection(targetOrientation, moveForward);
            }

            const double desiredWallDistance = initialFrontWallDistance  + targetDistance;

            // TODO: change the way to check the wall distance
            /* while (useWallDistance == true && hasTraveledWallDistance(desiredWallDistance, getWallDistance(VlxID::kFrontRight), moveForward) == false) {
                moveMotorsInADirection(targetOrientation, moveForward);
            } */

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

    resetWithBackWall(targetOrientation, currentOrientation, moveForward, useWallDistance);
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
    while (abs(pidTurn_.computeErrorOrientation(targetOrientation, currentOrientation)) > kMaxOrientationError) {
        #if DEBUG_MOVEMENT
        customPrintln("ErrorOrientation:" + String(pidTurn_.computeErrorOrientation(targetOrientation, currentOrientation)));
        #endif
        const unsigned long timeDiff = millis() - timePrev_;
        if (timeDiff < sampleTime_) {
            currentOrientation = bno_.getOrientationX();
            continue;
        }

        pidTurn_.computeTurn(targetOrientation, currentOrientation, speed, turnLeft);
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

bool Movement::hasTraveledDistanceWithSpeedForBackward(const double distance) {
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
    
    if (allDistanceTraveled_ >= distance) {
        allDistanceTraveled_ = 0;
        return true;
    }
    
    return false;
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

    /* if (allDistanceTraveled_ >= distance) {
        return true;
    } */
    
    return false;
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

    if (allDistanceTraveled_ >= distance) {
        return true;
    }
    
    return false;
}

bool Movement::hasTraveledWallDistance(double targetDistance, double currentDistance) {
    const double distanceDiff = (targetDistance - currentDistance);
    //moveForward = distanceDiff < 0;
    // customPrintln("DistanceDiff:" + String(distanceDiff));
    // vlxDistanceTraveled_ =  initialVlxDistance - currentDistance;
    // customPrint("RETURN:" + String(abs(distanceDiff) < kMaxDistanceError));
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

double Movement::weightMovemnt(double currentDistanceBack, double currentDistanceFront, double initialVlxDistanceBack, double initialVlxDistanceFront) {
    double vlxDistanceTraveled = 0;
    
    if (currentDistanceBack > kUnreachableDistance && currentDistanceFront > kUnreachableDistance) {
       return allDistanceTraveled_;
    }
    if (currentDistanceBack < currentDistanceFront) {
        vlxDistanceTraveled = currentDistanceBack - initialVlxDistanceBack;
    } else {
        vlxDistanceTraveled = initialVlxDistanceFront - currentDistanceFront;
    }
    return (allDistanceTraveled_ * kWeightEncoders + vlxDistanceTraveled * kWeightVlx);
    
}

bool Movement::centerInTile() {
    const double distanceBack = vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance();
    distanceToCenter_ = getDistanceToCenter();
    if (hasWallBehind()) {
        distanceToCenter_ = kTileLength - kLargeOfRobot - distanceToCenter_;
        distanceToCenter_ = distanceToCenter_ / kMToCm;
        return true;
    }

    return false;
}

bool Movement::hasWallBehind() {
    customPrintln("BackDistance:" + String(vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance()));
    
    return vlx[static_cast<uint8_t>(VlxID::kBack)].getRawDistance() <= wallBehindDistance_;
}

void Movement::resetWithBackWall(const double targetOrientation, double currentOrientation,bool moveForward ,bool useWallDistance){
    if (!inResetRoutine_ && counterMovements_ >= 4 && hasWallBehind() ) {
        stopMotors();
        inResetRoutine_ = true;
        
        moveMotors(MovementState::kBackward, getOrientation(currentOrientation), 0.15, useWallDistance);
        const double realOrientation = bno_.getOrientationX();
        double phaseCorrection = realOrientation - targetOrientation;
        customPrintln("currentOrientation:" + String(currentOrientation));
        customPrintln("realOrientation:" + String(realOrientation));
        // check if the second time the robot makes the update of the orientation
        if (abs(phaseCorrection) < 180) {
            customPrintln("1");
            phaseCorrection = phaseCorrection;
        } else if (realOrientation < currentOrientation) {
            customPrintln("2");
            phaseCorrection = 360 - abs(phaseCorrection);
        } else {
            customPrintln("3");
            phaseCorrection = abs(phaseCorrection) - 360;
        }
        customPrintln("PhaseCorrection:" + String(phaseCorrection));
        bno_.setPhaseCorrection(phaseCorrection);
        encodersReset_ = true;
        allDistanceTraveled_ = 0;
        
        if (centerInTile() == true) {
            moveForward = true;
            moveMotors(MovementState::kForward, targetOrientation, distanceToCenter_, moveForward);
        }
        
        inResetRoutine_ = false;
        counterMovements_ = 0;
    }

}