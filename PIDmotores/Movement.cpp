
#include "Movement.h"
#include "Pins.h"
#include "CustomSerial.h"
#include "VLX.h"

#define DEBUG_MOVEMENT 0

Movement::Movement() {
    this->prevTimeTraveled_ = millis();
    this->motor[kNumberOfWheels];
    this->currentState_;
    this->pidForward_.setTunnings(kPForward, kIForward, kDForward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidBackward_.setTunnings(kPBackward, kIBackward, kDBackward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidTurn_.setTunnings(kPTurn, kITurn, kDTurn, kTurnMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedTurn_, kMaxOrientationError);
}

void Movement::setup() {

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
    double distanceLeft = 0;
    double distanceRight = 0;
    distanceLeft = wallDistances[static_cast<uint8_t>(VlxID::kFrontLeft)];
    distanceRight = wallDistances[static_cast<uint8_t>(VlxID::kFrontRight)];
    distanceLeft *= kMToCm;
    double distanceToCenter = ((uint8_t)distanceLeft % kTileLength) - kVlxOffset;
    return distanceToCenter / kMToCm;
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
    // target= currentAngle +deltaAngle 
    // target = target < 0? 360 + target: target % 360;
    moveMotors(MovementState::kTurnRight, 90, 0);
}

void Movement::moveMotors(const MovementState state, const double targetOrientation, const double targetDistance) {
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

    getAllWallsDistances(&wallDistances[kNumberOfVlx]);

    const uint8_t initialFrontWallDistance = wallDistances[static_cast<uint8_t>(VlxID::kFrontRight)];
    
    bool moveForward = false;
    switch (state)
    {
        case (MovementState::kStop): {
            stopMotors();
            break;
        }
        case (MovementState::kForward): {
            moveForward = true;
            while (hasTraveledDistanceWithSpeed(targetDistance) == false){
                moveMotorsInADirection(targetOrientation, moveForward);

                if (limitSwitch_[0].getState(LimitSwitchID::kLeft) == true && limitSwitch_[1].getState(LimitSwitchID::kRight) == false) {
                    saveLastState(currentState_);
                    moveMotors(MovementState::kTurnLeft, currentOrientation + deltaOrientation_, 0);
                    moveMotors(MovementState::kBackward, 0, 0.1);
                    moveMotors(MovementState::kTurnRight, currentOrientation - deltaOrientation_, 0);
                    // Then return to the previous state 

                    //moveMotors(lastState_, currentOrientation, lastTargetDistance_);
                    
                    /* 
                    Something like this 
                    moveMotors(MovementState::kTurnLeft, currentOrientation + 5, 0);
                    moveMotors(MovementState::kBackward, 0, 0.1);
                    moveMotors(MovementState::kTurnRight, currentOrientation - 5, 0);
                    then return to the previous state
                    */
                } else if (limitSwitch_[0].getState(LimitSwitchID::kLeft) == false && limitSwitch_[1].getState(LimitSwitchID::kLeft) == true) {

                    // Turn a few angles to the right
                    /* 
                    Something like this 
                    moveMotors(MovementState::kTurnRight, currentOrientation - 5, 0);
                    moveMotors(MovementState::kBackward, 0, 0.1);
                    moveMotors(MovementState::kTurnLeft, currentOrientation + 5, 0);
                    then return to the previous state
                    */
                } else {
                    // Do nothing
                }
                /*  Step 1: 1
                        Check LimitSwitches
                    Step 2: 2
                        Left LimitSwitch on
                        Right LimitSwitch off
                    Step 3: 4
                        Save the LastState of the motors
                    Step 4; 3
                        Left turn few angles
                        Backward
                        Right turn few angles the same as left
                    Step 5; 5
                        Recover the LastState of the motors                     
                */
                checkWallsDistances();
            } 
            
            const double desiredWallDistance = initialFrontWallDistance - targetDistance;
            // TODO: Change the way to check the wall distance
            while (hasTraveledWallDistance(desiredWallDistance, getDistanceToCenter(), moveForward) == false) {
                moveMotorsInADirection(targetOrientation, moveForward);
            }

            stopMotors();
            
            break;
        }
        case (MovementState::kBackward): {
            moveForward = false;
            while (hasTraveledDistanceWithSpeed(targetDistance) == false) {
                moveMotorsInADirection(targetOrientation, moveForward);
            }

            const double desiredWallDistance = initialFrontWallDistance  + targetDistance;

            // TODO: change the way to check the wall distance
            while (hasTraveledWallDistance(desiredWallDistance, getWallDistance(VlxID::kFrontRight), moveForward) == false) {
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
    }
}

void Movement::turnMotors(const double targetOrientation, const double targetDistance, double &currentOrientation){
    double speeds[kNumberOfWheels];
    MotorState directions[kNumberOfWheels]; 
    double speed = 0;
    bool turnLeft = false;
    while (abs(pidTurn_.computeErrorOrientation(targetOrientation, currentOrientation)) > kMaxOrientationError) {
        #if DEBUG_MOVEMENT
        customPrintln("ErrorOrientation:" + String(pidTurn.computeErrorOrientation(targetOrientation, currentOrientation)));
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

void Movement::saveLastState(const MovementState state) {
    lastState_ = state;
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

bool Movement::hasTraveledDistanceWithSpeed(const double distance){
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

bool Movement::hasTraveledWallDistance(double targetDistance, double currentDistance, bool &moveForward) {
    const double distanceDiff = (targetDistance - currentDistance);
    moveForward = distanceDiff <= 0;

    return abs(distanceDiff) < kMaxDistanceError;
}