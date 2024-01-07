#include "PID.h"

PID::PID() {
    timePrev = millis();
}

PID::PID(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime) {
    timePrev = millis();
    double sampleTime_ = sampleTime;
    errorPrev = 0;
    maxError = maxErrorSum;
    double minOutput_ = minOutput;
    double maxOutput_ = maxOutput;
}

PID::PID(const double kP, const double kI, const double kD) {
    timePrev = millis();
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;

    errorPrev = 0;
}

double PID::computeErrorOrientation(const double targetOrientation, const double currentOrientation) {
    double errorOrientation = targetOrientation - currentOrientation;
    if (errorOrientation > 180) {
        errorOrientation -= 360;
    }
    else if (errorOrientation < -180) {
        errorOrientation += 360;
    }
    
    return errorOrientation;
}

double PID::computeOutputModifier(const double errorOrientation, const unsigned long timeDiff) {
    errorSum += errorOrientation * (timeDiff);
    const double errorDeriv = (errorOrientation - errorPrev) / (timeDiff);
    const double outputModifier = kP_ * errorOrientation + kI_ * errorSum + kD_ * errorDeriv;
    errorPrev = errorOrientation;
    return outputModifier;
}

void PID::computeStraight(const double targetOrientation, const double currentOrientation ,double &outputLeft, double &outputRight) {
    unsigned long timeDiff = millis() - timePrev;
    const double errorOrientation = computeErrorOrientation(targetOrientation, currentOrientation);
    const double outputModifier = computeOutputModifier(errorOrientation, timeDiff);
    constexpr int kBaseSpeed = 70;
    constexpr int kMaxModifier = 50;
    if (errorOrientation < 0) {
        outputLeft = kBaseSpeed + outputModifier;
        outputRight = kBaseSpeed - outputModifier;
        Serial.println("Aumentando derecho");
        Serial.println("OUTPUTMODIFIER:" + String(outputModifier));
    }
    else if (errorOrientation > 0) {
        outputRight = kBaseSpeed - outputModifier;
        outputLeft = kBaseSpeed + outputModifier;
        Serial.println("Aumentando izquierdo");
        Serial.println("OUTPUTMODIFIER:" + String(outputModifier));

    }
    else{
        outputLeft = kBaseSpeed;
        outputRight = kBaseSpeed;
        Serial.println("Manteniendo");
    }
    outputLeft = constrain(outputLeft, kBaseSpeed - kMaxModifier, kBaseSpeed + kMaxModifier);
    outputRight = constrain(outputRight, kBaseSpeed - kMaxModifier, kBaseSpeed + kMaxModifier);

    timePrev = millis(); 
}

void PID::compute(const double setpoint, double &input, double &output, int &resetVariable, const double pulsesPerRev, const double countTimeSampleInSec){
    if (millis() - timePrev < sampleTime) {
        return;
    }

    input = ((resetVariable / pulsesPerRev) * countTimeSampleInSec);
    resetVariable = 0;

    const double error = setpoint - input;
    output = error * kP_ + errorSum * kI_ + (error - errorPrev) * kD_;
    errorPrev = error;
    errorSum += error;

    errorSum = constrain(errorSum, minOutput, maxOutput);
    output = constrain(output, minOutput, maxOutput);

    timePrev = millis();
}

void PID::computeTurn(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight, bool &clockwise) {
    bool goalReached = false;
    unsigned long timeDiff = millis() - timePrev;
    const double errorOrientation = computeErrorOrientation(targetOrientation, currentOrientation);
    const double outputModifier = computeOutputModifier(errorOrientation, timeDiff);

    const int baseSpeed = 70;
    if (errorOrientation < 0) {
        outputLeft = baseSpeed + outputModifier;
        outputRight= baseSpeed + outputModifier;
        clockwise = true;
        Serial.println("Aumentando derecho");
        Serial.println("OUTPUTMODIFIER:" + String(outputModifier));
    }
    else if (errorOrientation > 0) {
        outputRight = baseSpeed - outputModifier;
        outputLeft= baseSpeed - outputModifier;
        clockwise = false;
        Serial.println("Aumentando izquierdo");
        Serial.println("OUTPUTMODIFIER:" + String(outputModifier));
    }
    else { 
        outputLeft = 0;
        outputRight = 0;
        goalReached = true;
    }
    if (goalReached == false) {
        outputLeft = constrain(outputLeft, 40, 120);
        outputRight = constrain(outputRight, 40, 120);
    }

    errorPrev = errorOrientation;

    timePrev = millis();
}