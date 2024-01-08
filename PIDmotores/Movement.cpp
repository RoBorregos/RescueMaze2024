
#include "Movement.h"
#include "Pins.h"
#include "Encoder.h"

constexpr double kPForward = 1.5; 
constexpr double kIForward = 0.3;
constexpr double kDForward = 0.0;

constexpr double kPBackward = 1.0;
constexpr double kIBackward = 0.0;
constexpr double kDBackward = 0.0;

constexpr double kPTurn = 1.0;
constexpr double kITurn = 0.0;
constexpr double kDTurn = 0.0;

PID pidForward(kPForward, kIForward, kDForward);
PID pidBackward(kPBackward, kIBackward, kDBackward);
PID pidTurn(kPTurn, kITurn, kDTurn);
BNO bno;

Movement::Movement() {
    this->motor[4];

}

void Movement::setup() {
    setupInternal(MotorID::kFrontLeft);
    setupInternal(MotorID::kFrontRight);
    setupInternal(MotorID::kBackLeft);
    setupInternal(MotorID::kBackRight);
    bno.setupBNO();
    Encoder::initEncoder();
}

void Movement::setupInternal(MotorID motorId) {
    int index = static_cast<int>(motorId);
    motor[index].motorSetup(
        pwmPin[index],
        digitalOne[index],
        digitalTwo[index],
        encoderA[index],
        motorId);
}

void Movement::stopMotors() {
    for(int i = 0; i < 4; ++i){
        motor[i].motorStop();
    }
}

void Movement::setPwmsAndDirections(const uint8_t pwms[4], const MotorState directions[4]) {
    for(int i = 0; i < 4; ++i){
        motor[i].setPwmAndDirection(pwms[i], directions[i]);
    }
}

void Movement::setMotorsDirections(const MovementState state, MotorState directions[4]) {
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
    uint8_t pwms[4];
    MotorState directions[4];
    int pwm = 60; // TODO: velocidad metros por segundos
    double currentOrientation = bno.getOrientationX();
    double pwmLeft = 0;
    double pwmRight = 0;
    bool turnLeft = false;
    const int frontLeftIndex = static_cast<int>(MotorID::kFrontLeft);
    const int frontRightIndex = static_cast<int>(MotorID::kFrontRight);
    const int backLeftIndex = static_cast<int>(MotorID::kBackLeft);
    const int backRightIndex = static_cast<int>(MotorID::kBackRight);
    constexpr double kMaxOrientationError = 0.3;

    switch (state)
    {
        case (MovementState::kStop): {
            stopMotors();
            break;
        }
        case (MovementState::kForward): {
            pidForward.computeStraight(targetOrientation, currentOrientation, pwmLeft, pwmRight);

            pwms[frontLeftIndex] = pwmLeft;
            pwms[backLeftIndex] = pwmLeft;
            pwms[frontRightIndex] = pwmRight;
            pwms[backRightIndex] = pwmRight;

            setMotorsDirections(MovementState::kForward, directions);
            setPwmsAndDirections(pwms, directions);
            
            break;
        }
        case (MovementState::kBackward): {
            pidBackward.computeStraight(targetOrientation, currentOrientation, pwmLeft, pwmRight);

            pwms[frontLeftIndex] = pwmRight;
            pwms[backLeftIndex] = pwmRight;
            pwms[frontRightIndex] = pwmLeft;
            pwms[backRightIndex] = pwmLeft;

            setMotorsDirections(MovementState::kBackward, directions); 
            setPwmsAndDirections(pwms, directions);
            
            break;
        }
        // TODO: change MotorStarte of turnRigth and left to make an oneself motorState and with that I mean turn 

        case (MovementState::kTurnLeft): {
            while (abs(pidTurn.computeErrorOrientation(targetOrientation, currentOrientation)) > kMaxOrientationError) {
                Serial.println(abs(targetOrientation - currentOrientation));

                pidTurn.computeTurn(targetOrientation, currentOrientation, pwmLeft, pwmRight, turnLeft);

                if (turnLeft) {
                    setMotorsDirections(MovementState::kTurnLeft, directions); 
                }
                else {
                    setMotorsDirections(MovementState::kTurnRight, directions); 
                }

                pwms[frontLeftIndex] = pwmLeft;
                pwms[backLeftIndex] = pwmLeft;
                pwms[frontRightIndex] = pwmRight;
                pwms[backRightIndex] = pwmRight;

                currentOrientation = bno.getOrientationX();
                setPwmsAndDirections(pwms, directions);
            }
            stopMotors();

            break;
        }
        case (MovementState::kTurnRight): {
            while (abs(pidTurn.computeErrorOrientation(targetOrientation, currentOrientation)) > kMaxOrientationError) {
                pidTurn.computeTurn(targetOrientation, currentOrientation, pwmLeft, pwmRight, turnLeft);

                if (turnLeft) {
                    setMotorsDirections(MovementState::kTurnLeft, directions); 
                }
                else {
                    setMotorsDirections(MovementState::kTurnRight, directions); 
                }

                pwms[frontLeftIndex] = pwmLeft;
                pwms[backLeftIndex] = pwmLeft;
                pwms[frontRightIndex] = pwmRight;
                pwms[backRightIndex] = pwmRight;

                currentOrientation = bno.getOrientationX();
                setPwmsAndDirections(pwms, directions);
            }
            stopMotors();

            break;
        }
    }
}


// CALIZZ ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
void Movement::updateTics(MotorID motorId) {

    int index = static_cast<int>(motorId);
    if (index >= 0 && index < 4) {
        motor[index].deltaTics(1);
        if (motor[index].getCurrentState() == MotorState::kForward){
            motor[index].deltaTotalTics(1);
        } else if (motor[index].getCurrentState() == MotorState::kBackward){
            motor[index].deltaTotalTics(-1);
        }
        else {
            return;
        }
    }

} 

double Movement::getBackLeftEncoderTics() {
    return motor[static_cast<int>(MotorID::kBackLeft)].ticsToMs();
}

double Movement::getFrontLeftEncoderTics() {
    return motor[static_cast<int>(MotorID::kFrontLeft)].ticsToMs();
}

double Movement::getBackRightEncoderTics() {
    return motor[static_cast<int>(MotorID::kBackRight)].ticsToMs();
}

double Movement::getFrontRightEncoderTics() {
    return motor[static_cast<int>(MotorID::kFrontRight)].ticsToMs();
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

