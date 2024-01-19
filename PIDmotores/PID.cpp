#include "PID.h"
#include "CustomSerial.h"

PID::PID() {
    timePrev_ = millis();
    errorPrev_ = 0;
}

PID::PID(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime) {
    timePrev_ = millis();
    errorPrev_ = 0;
    setTunnings(kP_, kI_, kD_, minOutput_, maxOutput_, maxErrorSum_, sampleTime_);
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
    constexpr int kBaseSpeed = 70;
    constexpr int kMaxModifier = 50;
    if (errorOrientation < 0) {
        outputLeft = kBaseSpeed + outputModifier;
        outputRight = kBaseSpeed - outputModifier;
        customPrintln("Aumentando derecho");
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
    }
    else if (errorOrientation > 0) {
        outputRight = kBaseSpeed - outputModifier;
        outputLeft = kBaseSpeed + outputModifier;
        customPrintln("Aumentando izquierdo");
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));

    }
    else{
        outputLeft = kBaseSpeed;
        outputRight = kBaseSpeed;
        customPrintln("Manteniendo");
    }
    outputLeft = constrain(outputLeft, kBaseSpeed - kMaxModifier, kBaseSpeed + kMaxModifier);
    outputRight = constrain(outputRight, kBaseSpeed - kMaxModifier, kBaseSpeed + kMaxModifier);

    timePrev_ = millis(); 
}
/* void PID::computeStraight(const double targetOrientation, const double currentOrientation ,double &outputLeft, double &outputRight) {
    unsigned long timeDiff = millis() - timePrev_;
    const double errorOrientation = computeErrorOrientation(targetOrientation, currentOrientation);
    const double outputModifier = computeOutputModifier(errorOrientation, timeDiff);
    constexpr int kBaseSpeed = motor.constantSpeed(0.05);
    constexpr int kSpeedModifier = outputModifier;
    constexpr int kMaxModifier = 0.30;
    if (errorOrientation < 0) {
        outputLeft = kBaseSpeed + kSpeedModifier;
        outputRight = kBaseSpeed - kSpeedModifier;
        customPrintln("Aumentando derecho");
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
    }
    else if (errorOrientation > 0) {
        outputRight = kBaseSpeed - kSpeedModifier;
        outputLeft = kBaseSpeed + kSpeedModifier;
        customPrintln("Aumentando izquierdo");
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));

    }
    else{
        outputLeft = kBaseSpeed;
        outputRight = kBaseSpeed;
        customPrintln("Manteniendo");
    }
    outputLeft = constrain(outputLeft, kBaseSpeed - kMaxModifier, kBaseSpeed + kMaxModifier);
    outputRight = constrain(outputRight, kBaseSpeed - kMaxModifier, kBaseSpeed + kMaxModifier);

    timePrev_ = millis(); 
} */

void PID::computeTurn(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight, bool &clockwise) {
    bool goalReached = false;
    unsigned long timeDiff = millis() - timePrev_;
    const double errorOrientation = computeErrorOrientation(targetOrientation, currentOrientation);
    const double outputModifier = computeOutputModifier(errorOrientation, timeDiff);

    const int baseSpeed = 70;
    if (errorOrientation < 0) {
        outputLeft = baseSpeed + outputModifier;
        outputRight= baseSpeed + outputModifier;
        clockwise = true;
        customPrintln("Aumentando derecho");
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
    }
    else if (errorOrientation > 0) {
        outputRight = baseSpeed - outputModifier;
        outputLeft= baseSpeed - outputModifier;
        clockwise = false;
        customPrintln("Aumentando izquierdo");
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
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
    errorSum_ = constrain(errorSum_, 4000 * -1, 4000);
    output = constrain(output, 0, 255);
    resetVariable = 0;
    timePrev_ = millis();
}