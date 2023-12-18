#include "PID.h"

PID::PID() {
    timePrev = millis();
}

PID::PID(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime) {
    timePrev = millis();
    //setTunings(kP, kI, kD);
    double sampleTime_ = sampleTime;
    errorPrev = 0;
    maxError = maxErrorSum;
    double minOutput_ = minOutput;
    double maxOutput_ = maxOutput;
}

PID::PID(const double kP, const double kI, const double kD) {
    timePrev = millis();
    //setTunings(kP, kI, kD);
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
    const double outputModifier = kP * errorOrientation + kI * errorSum + kD * errorDeriv;
    errorPrev = errorOrientation;
    return outputModifier;
}

void PID::computeStraight(const double targetOrientation, const double currentOrientation ,double &outputLeft, double &outputRight) {
    unsigned long timeDiff = millis() - timePrev;
    double errorOrientation;
    double outputModifier;
    computeErrorOrientation(targetOrientation, currentOrientation);

    computeOutputModifier(errorOrientation, timeDiff);
    const int baseSpeed = 70; 
    if (errorOrientation < 0) {
        outputLeft = baseSpeed + outputModifier;
        outputRight= baseSpeed - outputModifier;
        Serial.println("Aumentando derecho");
        Serial.println("OUTPUTMODIFIER:" + String(outputModifier));
    }
    else if (errorOrientation > 0) {
        outputRight = baseSpeed - outputModifier;
        outputLeft= baseSpeed + outputModifier;
        Serial.println("Aumentando izquierdo");
        Serial.println("OUTPUTMODIFIER:" + String(outputModifier));

    }
    else{
        outputLeft = baseSpeed;
        outputRight = baseSpeed;
        Serial.println("Manteniendo");
    }
    outputLeft = constrain(outputLeft, 40, 120);
    outputRight = constrain(outputRight, 40, 120);

    Serial.println(outputLeft);
    Serial.println(outputRight);

    timePrev = millis(); 
}

void PID::computeTurn(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight, bool &clockwise) {
    bool goalReached = false;
    unsigned long timeDiff = millis() - timePrev;
    double errorOrientation;
    double outputModifier;
    computeErrorOrientation(targetOrientation, currentOrientation);

    computeOutputModifier(errorOrientation, timeDiff);
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