#include "PID.h"
#include "CustomSerial.h"


PID::PID() {
    timePrev_ = millis();
    errorPrev_ = 0;
}

PID::PID(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime) {
    timePrev_ = millis();
    errorPrev_ = 0;
    setTunnings(kP, kI, kD, minOutput, maxOutput, maxErrorSum, sampleTime);
}

PID::PID(const double kP, const double kI, const double kD) {
    timePrev_ = millis();
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;

    errorPrev_ = 0;
}

void PID::setTunnings(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime) {
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
    minOutput_ = minOutput;
    maxOutput_ = maxOutput;
    maxErrorSum_ = maxErrorSum;
    sampleTime_ = sampleTime;
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

double PID::computeOutputModifier(const double error, const unsigned long timeDiff) {
    // TODO: Check if timeDiff is needed in the errorSum_ calculation
    errorSum_ += error * (timeDiff);
    errorSum_ = constrain(errorSum_, maxErrorSum_ * -1, maxErrorSum_);
    const double errorDeriv = (error - errorPrev_) / (timeDiff);
    const double outputModifier = kP_ * error + kI_ * errorSum_ + kD_ * errorDeriv;
    errorPrev_ = error;
    return outputModifier;
}

void PID::computeStraight(const double targetOrientation, const double currentOrientation ,double &outputLeft, double &outputRight) {
    unsigned long timeDiff = millis() - timePrev_;
    const double errorOrientation = computeErrorOrientation(targetOrientation, currentOrientation);
    const double outputModifier = computeOutputModifier(errorOrientation, timeDiff);
    constexpr double kBaseSpeed = 0.14;
    if (errorOrientation < 0) {
        outputLeft = kBaseSpeed + outputModifier;
        outputRight = kBaseSpeed - outputModifier;
    }
    else if (errorOrientation > 0) {
        outputRight = kBaseSpeed - outputModifier;
        outputLeft = kBaseSpeed + outputModifier;

    }
    else{
        outputLeft = kBaseSpeed;
        outputRight = kBaseSpeed;
    }
    outputLeft = constrain(outputLeft, minOutput_, maxOutput_);
    outputRight = constrain(outputRight, minOutput_, maxOutput_);

    timePrev_ = millis(); 
}

void PID::computeTurn(const double targetOrientation, const double currentOrientation, double &speed, bool &clockwise) {
    customPrintln("ENTRANDO A COMPUTETURN");
    bool goalReached = false;
    unsigned long timeDiff = millis() - timePrev_;
    const double errorOrientation = computeErrorOrientation(targetOrientation, currentOrientation);
    const double outputModifier = computeOutputModifier(errorOrientation, timeDiff);

    const double baseSpeed = 0.14;

    if (errorOrientation < 0) {
        speed = baseSpeed + outputModifier;
        clockwise = true;
        customPrintln("Aumentando derecho");
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
    }
    else if (errorOrientation > 0) {
        speed = baseSpeed - outputModifier;
        clockwise = false;
        customPrintln("Aumentando izquierdo");
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
    }
    else { 
        speed = 0;
        goalReached = true;
    }
    if (goalReached == false) {
        speed = constrain(speed, minOutput_, maxOutput_);
    }

    errorPrev_ = errorOrientation;

    timePrev_ = millis();
}

void PID::compute(const double setpoint, double& input, double& output, long long &resetVariable, double (*func)(const long long, const unsigned long)) {
    if(millis() - timePrev_ < sampleTime_) {
        return;
    }
    
    input = func(resetVariable, sampleTime_);

    // TODO: Call the method computeOutputModifier to replace the line 118 - 124
    const double error = setpoint - input;
    output = error * kP_ + errorSum_ * kI_ + (error - errorPrev_) * kD_;

    errorPrev_ = error;
    errorSum_ += error;
    errorSum_ = constrain(errorSum_, maxErrorSum_ * -1, maxErrorSum_);
    output = constrain(output, minOutput_, maxOutput_);
    resetVariable = 0;
    timePrev_ = millis();
}