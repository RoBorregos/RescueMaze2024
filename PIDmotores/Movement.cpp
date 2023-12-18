
#include "Movement.h"
#include "Pins.h"
//#include "Encoder.h"

constexpr double kPStraight = 5.0; 
constexpr double kIStraight = 0.008;
constexpr double kDStraight = 0.0;

constexpr double kPTurn = 1.0;
constexpr double kITurn = 0.0;
constexpr double kDTurn = 0.0;

PID pidStraight(kPStraight, kIStraight, kDStraight);
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
    //Encoder::initEncoder();
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

void Movement::forwardMotors(const uint8_t pwms[4]) {
    for(int i = 0; i < 4; ++i){
        motor[i].motorForward(pwms[i]);
    }
}

void Movement::backwardMotors(const uint8_t pwms[4]) {
    for(int i = 0; i < 4; ++i){
        motor[i].motorBackward(pwms[i]);
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
    int pwm = 60;
    double currentOrientation = bno.getOrientationX();
    double pwmLeft = 0;
    double pwmRight = 0;
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
            pidStraight.computeStraight(targetOrientation, currentOrientation, pwmLeft, pwmRight);

            pwms[frontLeftIndex] = pwmLeft;
            pwms[backLeftIndex] = pwmLeft;
            pwms[frontRightIndex] = pwmRight;
            pwms[backRightIndex] = pwmRight;

            setMotorsDirections(state, directions);            
            break;
        }
        case (MovementState::kBackward): {
            pwms[frontLeftIndex] = pwm;
            pwms[backLeftIndex] = pwm;
            pwms[frontRightIndex] = pwm;
            pwms[backRightIndex] = pwm;

            setMotorsDirections(state, directions); 
            break;
        }
        // TODO: cambiar el MotorState de turnRigth y left para que solo sea uno motorState es decir turn

        case (MovementState::kTurnLeft): {
            while (targetOrientation != currentOrientation) {
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

            break;
        }
        case (MovementState::kTurnRight): {
            while (targetOrientation != currentOrientation) {
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

            break;
        }
    }
    setPwmsAndDirections(pwms, directions);
}


// CALIZZ ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''FEDFE'FREFEJ,FBEJFFBHUWBFUHEWBFHJWERBFHJRBEJHFBRJFBKWRJNSBJFBKREJFBKEJB
void Movement::updateTics(MotorID motorId) {

    int index = static_cast<int>(motorId);
    if (index >= 0 && index < 4) {
        motor[index].deltaPidTics(1);
        if (motor[index].getCurrentState() == MotorState::kForward){
            motor[index].deltaEncoderTics(1);
        } else if (motor[index].getCurrentState() == MotorState::kBackward){
            motor[index].deltaEncoderTics(-1);
        }
        else {
            return;
        }
    }
    /* motor->deltaPidTics(1);

    if (motor->getCurrentState() == MotorState::kForward){
        motor->deltaEncoderTics(1);
    } else if (motor->getCurrentState() == MotorState::kBackward){
        motor->deltaEncoderTics(-1);
    }
    else {
        return;
    } */
} 

int Movement::getBackLeftEncoderTics() {
    return motor[static_cast<int>(MotorID::kBackLeft)].getEncoderTics();
}

int Movement::getFrontLeftEncoderTics() {
    return motor[static_cast<int>(MotorID::kFrontLeft)].getEncoderTics();
}

int Movement::getBackRightEncoderTics() {
    return motor[static_cast<int>(MotorID::kBackRight)].getEncoderTics();
}

int Movement::getFrontRightEncoderTics() {
    return motor[static_cast<int>(MotorID::kFrontRight)].getEncoderTics();
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



/* void Movement::computeTargetOrientation(compass targetOrientation, compass currentOrientation) {
    int target = getOrientation(targetOrientation);
    int current = getOrientation(currentOrientation);
    int error = target - current;
    if (error > 180) {
        error = error - 360;
    } else if (error < -180) {
        error = error + 360;
    }
    Serial.print("Error: ");
    Serial.println(error);
    if (error > 0) {
        moveMotors(MovementState::kForward);
    } else if (error < 0) {
        moveMotors(MovementState::kBackward);
    } else {
        moveMotors(MovementState::kStop);
    }
} */



