
#include "Movement.h"
#include "Pins.h"
#include "CustomSerial.h"

// TODO: ALL THIS VARIABLES SHOULD BE IN A CLASS AS CONSTANTS
constexpr static double kPBackward = 1.0;
constexpr static double kIBackward = 0.0;
constexpr static double kDBackward = 0.0;

constexpr static double kPTurn = 1.0;
constexpr static double kITurn = 0.0;
constexpr static double kDTurn = 0.0;

PID pidBackward(kPBackward, kIBackward, kDBackward);
PID pidTurn(kPTurn, kITurn, kDTurn);
BNO bno;

Movement::Movement() {
    this->motor[kNumberOfWheels];
    this->pidForward.setTunnings(kPForward, kIForward, kDForward, kMinOutput, kMaxOutput, kMaxErrorSum, kSampleTime);
}

void Movement::setup() {
    setupInternal(MotorID::kFrontLeft);
    setupInternal(MotorID::kFrontRight);
    setupInternal(MotorID::kBackLeft);
    setupInternal(MotorID::kBackRight);
    bno.setupBNO();
    
}

void Movement::setupInternal(MotorID motorId) {
    int index = static_cast<int>(motorId);
    motor[index].motorSetup(
        Pins::pwmPin[index],
        Pins::digitalOne[index],
        Pins::digitalTwo[index],
        Pins::encoderA[index],
        motorId);
}

void Movement::stopMotors() {
    for(int i = 0; i < kNumberOfWheels; ++i){
        motor[i].motorStop(0);
    }
}

// TODO: It will be needed to change this function to make it work with moveMotors
void Movement::setSpeed(const double speed) { // Speed in meters per second
    for(int i = 0; i < kNumberOfWheels; ++i){
        motor[i].constantSpeed(speed, MotorState::kForward);
    }
}

void Movement::setPwmsAndDirections(const uint8_t pwms[kNumberOfWheels], const MotorState directions[kNumberOfWheels]) {
    for (int i = 0; i < kNumberOfWheels; ++i) {
        motor[i].setPwmAndDirection(pwms[i], directions[i]);
    }
}

void Movement::setSpeedsAndDirections(const double speeds[kNumberOfWheels], const MotorState directions[kNumberOfWheels]) {
    for (int i = 0; i < kNumberOfWheels; ++i) {
        motor[i].setSpeedAndDirection(speeds[i], directions[i]);
    }
}

void Movement::setMotorsDirections(const MovementState state, MotorState directions[kNumberOfWheels]) {
    const int frontLeftIndex = static_cast<int>(MotorID::kFrontLeft);
    const int frontRightIndex = static_cast<int>(MotorID::kFrontRight);
    const int backLeftIndex = static_cast<int>(MotorID::kBackLeft);
    const int backRightIndex = static_cast<int>(MotorID::kBackRight);
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

void Movement::moveMotors(const MovementState state, const double targetOrientation) {
    double speeds[kNumberOfWheels];
    MotorState directions[kNumberOfWheels];
    int pwm = 60; 
    double currentOrientation = bno.getOrientationX();
    double speedLeft = 0;
    double speedRight = 0;
    bool turnLeft = false;
    const int frontLeftIndex = static_cast<int>(MotorID::kFrontLeft);
    const int frontRightIndex = static_cast<int>(MotorID::kFrontRight);
    const int backLeftIndex = static_cast<int>(MotorID::kBackLeft);
    const int backRightIndex = static_cast<int>(MotorID::kBackRight);

    switch (state)
    {
        case (MovementState::kStop): {
            stopMotors();
            break;
        }
        case (MovementState::kForward): {
            pidForward.computeStraight(targetOrientation, currentOrientation, speedLeft, speedRight);
            speeds[frontLeftIndex] = speedLeft;
            speeds[backLeftIndex] = speedLeft;
            speeds[frontRightIndex] = speedRight;
            speeds[backRightIndex] = speedRight;

            setMotorsDirections(MovementState::kForward, directions);
            setSpeedsAndDirections(speeds, directions);
            
            break;
        }
        // TODO: apply giving a speed to the motors to go backward
        case (MovementState::kBackward): {
            pidBackward.computeStraight(targetOrientation, currentOrientation, speedLeft, speedRight);

            speeds[frontLeftIndex] = speedRight;
            speeds[backLeftIndex] = speedRight;
            speeds[frontRightIndex] = speedLeft;
            speeds[backRightIndex] = speedLeft;

            setMotorsDirections(MovementState::kBackward, directions); 
            setSpeedsAndDirections(speeds, directions);
            
            break;
        }
        // TODO: change MotorStarte of turnRigth and left to make an oneself motorState and with that I mean turn 
        // TODO: apply giving a speed to the motors to turn left or right
        case (MovementState::kTurnLeft): {
            while (abs(pidTurn.computeErrorOrientation(targetOrientation, currentOrientation)) > kMaxOrientationError) {
                customPrintln(abs(targetOrientation - currentOrientation));

                pidTurn.computeTurn(targetOrientation, currentOrientation, speedLeft, speedRight, turnLeft);

                if (turnLeft) {
                    setMotorsDirections(MovementState::kTurnLeft, directions); 
                }
                else {
                    setMotorsDirections(MovementState::kTurnRight, directions); 
                }

                speeds[frontLeftIndex] = speedLeft;
                speeds[backLeftIndex] = speedLeft;
                speeds[frontRightIndex] = speedRight;
                speeds[backRightIndex] = speedRight;

                currentOrientation = bno.getOrientationX();
                setSpeedsAndDirections(speeds, directions);
            }
            stopMotors();

            break;
        }
        case (MovementState::kTurnRight): {
            while (abs(pidTurn.computeErrorOrientation(targetOrientation, currentOrientation)) > kMaxOrientationError) {
                pidTurn.computeTurn(targetOrientation, currentOrientation, speedLeft, speedRight, turnLeft);

                if (turnLeft) {
                    setMotorsDirections(MovementState::kTurnLeft, directions); 
                }
                else {
                    setMotorsDirections(MovementState::kTurnRight, directions); 
                }

                speeds[frontLeftIndex] = speedLeft;
                speeds[backLeftIndex] = speedLeft;
                speeds[frontRightIndex] = speedRight;
                speeds[backRightIndex] = speedRight;

                currentOrientation = bno.getOrientationX();
                setSpeedsAndDirections(speeds, directions);
            }
            stopMotors();

            break;
        }
    }
}

void Movement::updateTics(MotorID motorId) {
    const int index = static_cast<int>(motorId);
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
    return motor[static_cast<int>(MotorID::kBackLeft)].getSpeed();
}

double Movement::getFrontLeftSpeed() {
    return motor[static_cast<int>(MotorID::kFrontLeft)].getSpeed();
}

double Movement::getBackRightSpeed() {
    return motor[static_cast<int>(MotorID::kBackRight)].getSpeed();
}

double Movement::getFrontRightSpeed() {
    return motor[static_cast<int>(MotorID::kFrontRight)].getSpeed();
}

int Movement::getOrientation(const compass currentOrientation) {
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

