#include "Movement.h"
#include "Pins.h"
#include "CustomSerial.h"
#include "VLX.h"

#define DEBUG_MOVEMENT 0

Movement::Movement() {
    this->prevTimeTraveled_ = millis();
    this->motor[kNumberOfWheels];
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

uint8_t Movement::getIndexFromArray(const int value, const int array[]) {
    
    for (uint8_t i = 0; i < kNumberOfTargetOrientations; ++i) {
        const int currentValue = array[i];
        if (currentValue == value) {
            return i;
        } 

    }
    return -1;
}

bool Movement::checkWallsDistances(const TileDirection targetTileDirection, const double currentOrientation) {
    bool offArray = getIndexFromArray(currentOrientation, kTargetOrientations) == kOffArray;
    if (offArray) {
        return false;
    }

    const uint8_t vlxIndex = (static_cast<uint8_t>(targetTileDirection) + getIndexFromArray(currentOrientation, kTargetOrientations)) % kTileDirections;
    const VlxID vlxID = static_cast<VlxID>(vlxIndex);
    return getWallDistance(vlxID) < kMinWallDistance;
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

void Movement::goForward(const int targetOrientation) {
    moveMotors(MovementState::kForward, targetOrientation, kOneTileDistance);
}

void Movement::goBackward(const int targetOrientation) {
    moveMotors(MovementState::kBackward, targetOrientation, kOneTileDistance);
}

void Movement::turnLeft(const int targetOrientation) {
    moveMotors(MovementState::kTurnLeft, targetOrientation, 0);
}

void Movement::turnRight(const int targetOrientation) {
    moveMotors(MovementState::kTurnRight, targetOrientation, 0);
}

void Movement::moveMotors(const MovementState state, const int targetOrientation, const int targetDistance) {
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
            while (abs(pidTurn_.computeErrorOrientation(targetOrientation, currentOrientation)) > kMaxOrientationError) {
                #if DEBUG_MOVEMENT
                customPrintln("ErrorOrientation:" + String(pidTurn.computeErrorOrientation(targetOrientation, currentOrientation)));
                #endif
                const unsigned long timeDiff = millis() - timePrev_;
                if (timeDiff < sampleTime_) {
                    currentOrientation = bno_.getOrientationX();
                    continue;
                }
                //customPrintln(abs(targetOrientation - currentOrientation));

                pidTurn_.computeTurn(targetOrientation, currentOrientation, speedLeft, turnLeft);
                if (turnLeft) {
                    setMotorsDirections(MovementState::kTurnLeft, directions); 
                } else {
                    setMotorsDirections(MovementState::kTurnRight, directions); 
                }

                speeds[frontLeftIndex] = speedLeft;
                speeds[backLeftIndex] = speedLeft;
                speeds[frontRightIndex] = speedLeft;
                speeds[backRightIndex] = speedLeft;

                setSpeedsAndDirections(speeds, directions);
                
                timePrev_ = millis();
                
            }
            
            stopMotors();

            break;
        }
        case (MovementState::kTurnRight): {
            while (abs(pidTurn_.computeErrorOrientation(targetOrientation, currentOrientation)) > kMaxOrientationError) {
                #if DEBUG_MOVEMENT
                customPrintln("ErrorOrientation:" + String(pidTurn.computeErrorOrientation(targetOrientation, currentOrientation)));
                #endif
                const unsigned long timeDiff = millis() - timePrev_;
                if (timeDiff < sampleTime_) {
                    currentOrientation = bno_.getOrientationX();
                    continue;
                }
                //customPrintln(abs(targetOrientation - currentOrientation));

                pidTurn_.computeTurn(targetOrientation, currentOrientation, speedLeft, turnLeft);
                if (turnLeft) {
                    setMotorsDirections(MovementState::kTurnLeft, directions); 
                } else {
                    setMotorsDirections(MovementState::kTurnRight, directions); 
                }

                speeds[frontLeftIndex] = speedLeft;
                speeds[backLeftIndex] = speedLeft;
                speeds[frontRightIndex] = speedLeft;
                speeds[backRightIndex] = speedLeft;

                setSpeedsAndDirections(speeds, directions);

                timePrev_ = millis();
                
            }
            
            stopMotors();

            break;
        }
    }
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