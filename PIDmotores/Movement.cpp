
#include "Movement.h"
#include "Pins.h"
#include "CustomSerial.h"
#include "VLX.h"

BNO bno;
VLX vlx(Pins::vlxPins[0]);

Movement::Movement() {
    this->prevTimeTraveled_ = millis();
    this->motor[kNumberOfWheels];
    this->pidForward.setTunnings(kPForward, kIForward, kDForward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidBackward.setTunnings(kPBackward, kIBackward, kDBackward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedForward_, kMaxOrientationError);
    this->pidTurn.setTunnings(kPTurn, kITurn, kDTurn, kTurnMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime, kBaseSpeedTurn_, kMaxOrientationError);
}

void Movement::setup() {
    setupInternal(MotorID::kFrontLeft);
    setupInternal(MotorID::kFrontRight);
    setupInternal(MotorID::kBackLeft);
    setupInternal(MotorID::kBackRight);
    bno.setupBNO();
    vlx.init();
    
}

void Movement::setupInternal(MotorID motorId) {
    uint8_t index = static_cast<uint8_t>(motorId);
    motor[index].motorSetup(
        Pins::pwmPin[index],
        Pins::digitalOne[index],
        Pins::digitalTwo[index],
        Pins::encoderA[index],
        motorId);
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
void Movement::moveMotosForward(double targetOrientation){
    double speeds[kNumberOfWheels];
    MotorState directions[kNumberOfWheels]; 
    double currentOrientation = bno.getOrientationX();

    double speedLeft = 0;
    double speedRight = 0;

    const uint8_t frontLeftIndex = static_cast<uint8_t>(MotorID::kFrontLeft);
    const uint8_t frontRightIndex = static_cast<uint8_t>(MotorID::kFrontRight);
    const uint8_t backLeftIndex = static_cast<uint8_t>(MotorID::kBackLeft);
    const uint8_t backRightIndex = static_cast<uint8_t>(MotorID::kBackRight);

    const unsigned long timeDiff = millis() - timePrev_;
    if (timeDiff < sampleTime_) {
        currentOrientation = bno.getOrientationX();
        return;
    }
    pidForward.computeStraight(targetOrientation, currentOrientation, speedLeft, speedRight);
    speeds[frontLeftIndex] = speedLeft;
    speeds[backLeftIndex] = speedLeft;
    speeds[frontRightIndex] = speedRight;
    speeds[backRightIndex] = speedRight;
    customPrintln("SpeedLeft:" + String(speedLeft));
    customPrintln("SpeedRight:" + String(speedRight));

    setMotorsDirections(MovementState::kForward, directions);
    setSpeedsAndDirections(speeds, directions);

    //customPrintln(vlx.getDistance());

    timePrev_ = millis();
}

void Movement::moveMotorsBackward(double targetOrientation){
    double speeds[kNumberOfWheels];
    MotorState directions[kNumberOfWheels]; 
    double currentOrientation = bno.getOrientationX();

    double speedLeft = 0;
    double speedRight = 0;

    const uint8_t frontLeftIndex = static_cast<uint8_t>(MotorID::kFrontLeft);
    const uint8_t frontRightIndex = static_cast<uint8_t>(MotorID::kFrontRight);
    const uint8_t backLeftIndex = static_cast<uint8_t>(MotorID::kBackLeft);
    const uint8_t backRightIndex = static_cast<uint8_t>(MotorID::kBackRight);

    const unsigned long timeDiff = millis() - timePrev_;
    if (timeDiff < sampleTime_) {
        currentOrientation = bno.getOrientationX();
        return;
    }
    pidBackward.computeStraight(targetOrientation, currentOrientation, speedLeft, speedRight);
    speeds[frontLeftIndex] = speedRight;
    speeds[backLeftIndex] = speedRight;
    speeds[frontRightIndex] = speedLeft;
    speeds[backRightIndex] = speedLeft;

    setMotorsDirections(MovementState::kBackward, directions); 
    setSpeedsAndDirections(speeds, directions);
    timePrev_ = millis();
}

void Movement::moveMotors(const MovementState state, const double targetOrientation, const double targetDistance) {
    double speeds[kNumberOfWheels];
    MotorState directions[kNumberOfWheels]; 
    double currentOrientation = bno.getOrientationX();
    double speedLeft = 0;
    double speedRight = 0;
    bool turnLeft = false;
    const uint8_t frontLeftIndex = static_cast<uint8_t>(MotorID::kFrontLeft);
    const uint8_t frontRightIndex = static_cast<uint8_t>(MotorID::kFrontRight);
    const uint8_t backLeftIndex = static_cast<uint8_t>(MotorID::kBackLeft);
    const uint8_t backRightIndex = static_cast<uint8_t>(MotorID::kBackRight);
    double initialDistance = vlx.getDistance();
    bool moveForward = false;
    switch (state)
    {
        case (MovementState::kStop): {
            stopMotors();
            break;
        }
        case (MovementState::kForward): {
            while (hasTraveledDistanceWithSpeed(targetDistance) == false){
                moveMotosForward(targetOrientation);
            } 
            stopMotors();
            delay(1000);
            double desiredDistance = initialDistance - targetDistance;
            while (hasTraveledDistance(desiredDistance, vlx.getDistance(), moveForward) == false) {
                if (moveForward) {
                    moveMotosForward(targetOrientation);
                } else if (!moveForward) {
                    moveMotorsBackward(targetOrientation);
                }
            }

            stopMotors();
            
            break;
        }
        case (MovementState::kBackward): {
            while (hasTraveledDistanceWithSpeed(targetDistance) == false) {
                moveMotorsBackward(targetOrientation);
            }

            double desiredDistance = initialDistance + targetDistance;
            while (hasTraveledDistance(desiredDistance, vlx.getDistance(), moveForward) == false) {
                if (moveForward) {
                    moveMotosForward(targetOrientation);
                } else {
                    moveMotorsBackward(targetOrientation);
                }
            }
            
            stopMotors();
            
            break;
        }
        // TODO: change MotorStarte of turnRigth and left to make an oneself motorState and with that I mean turn 
        case (MovementState::kTurnLeft): {
            while (abs(pidTurn.computeErrorOrientation(targetOrientation, currentOrientation)) > kMaxOrientationError) {
                customPrintln("ErrorOrientation:" + String(pidTurn.computeErrorOrientation(targetOrientation, currentOrientation)));
                const unsigned long timeDiff = millis() - timePrev_;
                if (timeDiff < sampleTime_) {
                    currentOrientation = bno.getOrientationX();
                    continue;
                }
                //customPrintln(abs(targetOrientation - currentOrientation));

                pidTurn.computeTurn(targetOrientation, currentOrientation, speedLeft, turnLeft);
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
            while (abs(pidTurn.computeErrorOrientation(targetOrientation, currentOrientation)) > kMaxOrientationError) {
                customPrintln("ErrorOrientation:" + String(pidTurn.computeErrorOrientation(targetOrientation, currentOrientation)));
                const unsigned long timeDiff = millis() - timePrev_;
                if (timeDiff < sampleTime_) {
                    currentOrientation = bno.getOrientationX();
                    continue;
                }
                //customPrintln(abs(targetOrientation - currentOrientation));

                pidTurn.computeTurn(targetOrientation, currentOrientation, speedLeft, turnLeft);
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

bool Movement::hasTraveledDistance(double targetDistance, double currentDistance, bool &moveForward) {
    const double distanceDiff = (targetDistance - currentDistance);
    if (distanceDiff > 0) {
        moveForward = false;
    } else {
        moveForward = true;
    }
    if (abs(distanceDiff) < kMaxDistanceError) {
        return true;
    }

    return false;
}