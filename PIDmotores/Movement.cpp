
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

void Movement::turnRightMotors(const uint8_t pwms[4]) {
    
    motor[0].motorForward(pwms[0]);
    motor[1].motorForward(pwms[1]);
    motor[2].motorBackward(pwms[2]);
    motor[3].motorBackward(pwms[3]);
    
}

void Movement::forwardMotor(const uint8_t pwm, MotorID motorId) {
    int index = static_cast<int>(motorId);
    motor[index].setPwmAndDirection(pwm, MotorState::kForward);
}

void Movement::setPwmsAndDirections(const uint8_t pwms[4], const MotorState directions[4]) {
    for(int i = 0; i < 4; ++i){
        motor[i].setPwmAndDirection(pwms[i], directions[i]);
    }
}

void Movement::moveMotors(MovementState state, const double targetOrientation) {
    uint8_t pwms[4];
    MotorState direction[4];
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
            pidStraight.computeStraight(targetOrientation,currentOrientation, pwmLeft, pwmRight);

            pwms[frontLeftIndex] = pwmLeft;
            pwms[backLeftIndex] = pwmLeft;
            pwms[frontRightIndex] = pwmRight;
            pwms[backRightIndex] = pwmRight;

            direction[frontLeftIndex] = MotorState::kForward;
            direction[backLeftIndex] = MotorState::kForward;
            direction[frontRightIndex] = MotorState::kForward;
            direction[backRightIndex] = MotorState::kForward;
            break;
        }
        case (MovementState::kBackward): {
            pwms[frontLeftIndex] = pwm;
            pwms[backLeftIndex] = pwm;
            pwms[frontRightIndex] = pwm;
            pwms[backRightIndex] = pwm;

            direction[frontLeftIndex] = MotorState::kBackward;
            direction[backLeftIndex] = MotorState::kBackward;
            direction[frontRightIndex] = MotorState::kBackward;
            direction[backRightIndex] = MotorState::kBackward;
            break;
        }
        // TODO: cambiar el MotorState de turnRigth y left para que solo sea uno motorState es decir turn

        case (MovementState::kTurnLeft): {
            while (targetOrientation != currentOrientation) {
                pidTurn.computeTurn(targetOrientation,currentOrientation, pwmLeft, pwmRight, turnLeft);
                if (turnLeft) {
                    direction[frontLeftIndex] = MotorState::kBackward;
                    direction[backLeftIndex] = MotorState::kBackward;
                    direction[frontRightIndex] = MotorState::kForward;
                    direction[backRightIndex] = MotorState::kForward;
                }
                else {
                    direction[frontLeftIndex] = MotorState::kForward;
                    direction[backLeftIndex] = MotorState::kForward;
                    direction[frontRightIndex] = MotorState::kBackward;
                    direction[backRightIndex] = MotorState::kBackward;
                }

                pwms[frontLeftIndex] = pwmLeft;
                pwms[backLeftIndex] = pwmLeft;
                pwms[frontRightIndex] = pwmRight;
                pwms[backRightIndex] = pwmRight;

                currentOrientation = bno.getOrientationX();
                setPwmsAndDirections(pwms, direction);
            }

            break;
        }
        case (MovementState::kTurnRight): {
            pidTurn.computeTurn(targetOrientation,currentOrientation, pwmLeft, pwmRight, turnLeft);
            if (turnLeft) {
                direction[frontLeftIndex] = MotorState::kBackward;
                direction[backLeftIndex] = MotorState::kBackward;
                direction[frontRightIndex] = MotorState::kForward;
                direction[backRightIndex] = MotorState::kForward;
            }
            else {
                direction[frontLeftIndex] = MotorState::kForward;
                direction[backLeftIndex] = MotorState::kForward;
                direction[frontRightIndex] = MotorState::kBackward;
                direction[backRightIndex] = MotorState::kBackward;
            }

            pwms[frontLeftIndex] = pwmLeft;
            pwms[backLeftIndex] = pwmLeft;
            pwms[frontRightIndex] = pwmRight;
            pwms[backRightIndex] = pwmRight;

            break;
        }
    }
    setPwmsAndDirections(pwms, direction);
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
        case (compass::knorth): {
            return 0;
        }
        case (compass::keast): {
            return 90;
        }
        case (compass::ksouth): {
            return 180;
        }
        case (compass::kwest): {
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



