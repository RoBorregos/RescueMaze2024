#include "PID.h"
#include "CustomSerial.h"

PID::PID() {
    timePrev_ = millis();
    errorPrev_ = 0;

}

PID::PID(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime, double baseModifier, double kMaxOrientationError) {
    timePrev_ = millis();
    errorPrev_ = 0;
    kBaseModifier_ = baseModifier;
    kMaxOrientationError_ = kMaxOrientationError;
    setTunnings(kP, kI, kD, minOutput, maxOutput, maxErrorSum, sampleTime, baseModifier, kMaxOrientationError);
}

PID::PID(const double kP, const double kI, const double kD) {
    timePrev_ = millis();
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;

    errorPrev_ = 0;
}

void PID::setTunningsMotors(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime) {
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
    kMinOutput_ = minOutput;
    kMaxOutput_ = maxOutput;
    kMaxErrorSum_ = maxErrorSum;
    kSampleTime_ = sampleTime;
}

void PID::setTunnings(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime, double baseModifier, double kMaxOrientationError) {
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
    kMinOutput_ = minOutput;
    kMaxOutput_ = maxOutput;
    kMaxErrorSum_ = maxErrorSum;
    kSampleTime_ = sampleTime;
    kBaseModifier_ = baseModifier;
    kMaxOrientationError_ = kMaxOrientationError;
}

double PID::computeErrorOrientation(const double targetOrientation, const double currentOrientation) {
    double errorOrientation = targetOrientation - currentOrientation;
    if (errorOrientation > 180) {
        errorOrientation -= 360;
    } else if (errorOrientation < -180) {
        errorOrientation += 360;
    }
    
    return errorOrientation;
}

double PID::computeOutputModifier(const double error, const unsigned long timeDiff) {
    // TODO: Check if timeDiff is needed in the errorSum_ calculation
    errorSum_ += error;
    errorSum_ = constrain(errorSum_, kMaxErrorSum_ * -1, kMaxErrorSum_);
    const double errorDeriv = (error - errorPrev_) / (timeDiff);
    const double outputModifier = kP_ * error + kI_ * errorSum_ + kD_ * errorDeriv;
    errorPrev_ = error;
    customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
    return outputModifier;
}

void PID::computeStraight(const double targetOrientation, const double currentOrientation ,double &outputLeft, double &outputRight) {
    const unsigned long timeDiff = millis() - timePrev_;
    if (timeDiff < kSampleTime_) {
        customPrintln("TimeDiff:" + String(timeDiff));
        return;
    }
    const double errorOrientation = computeErrorOrientation(targetOrientation, currentOrientation);
    const double outputModifier = computeOutputModifier(errorOrientation, timeDiff);
    customPrintln("ERRORORIENTATION:" + String(errorOrientation));
    customPrintln("OUTPUTMODIFIER:" + String(outputModifier));

    if (errorOrientation < kMaxOrientationError_) {
        outputLeft = kBaseModifier_ + outputModifier;
        outputRight = kBaseModifier_ - outputModifier;
    }
    else if (errorOrientation > -kMaxOrientationError_) {
        outputRight = kBaseModifier_ - outputModifier;
        outputLeft = kBaseModifier_ + outputModifier;

    }
    else{
        outputLeft = kBaseModifier_;
        outputRight = kBaseModifier_;
    }
    customPrintln("outputLeft" + String(outputLeft));
    customPrintln("outputRight" + String(outputRight));
    customPrintln("baseModifier" + String(kBaseModifier_));
    outputLeft = constrain(outputLeft, kMinOutput_, kMaxOutput_);
    outputRight = constrain(outputRight, kMinOutput_, kMaxOutput_);
    errorPrev_ = errorOrientation;

    timePrev_ = millis(); 
}

void PID::computeTurn(const double targetOrientation, const double currentOrientation, double &speed, bool &clockwise) {
    const unsigned long timeDiff = millis() - timePrev_;
    if (timeDiff < kSampleTime_) {
        return;
    }
    
    bool goalReached = false;
    const double errorOrientation = computeErrorOrientation(targetOrientation, currentOrientation);
    const double outputModifier = computeOutputModifier(errorOrientation, timeDiff);

    if (errorOrientation < kMaxOrientationError_) {
        speed = kBaseModifier_ + outputModifier;
        clockwise = true;
        customPrintln("Aumentando derecho");
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
    } else if (errorOrientation > -kMaxOrientationError_) {
        speed = kBaseModifier_ - outputModifier;
        clockwise = false;
        customPrintln("Aumentando izquierdo");
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
    } else { 
        speed = 0;
        goalReached = true;
    }
    
    if (goalReached == false) {
        speed = constrain(speed, kMinOutput_, kMaxOutput_);
    }
    customPrintln("SPEED:" + String(speed));
    errorPrev_ = errorOrientation;

    timePrev_ = millis();
}

/* TODO:
void PID.computeDistance(....) {

    
    baseModifier

}
 */

void PID::compute(const double setpoint, double& input, double& output, long long &resetVariable, double (*func)(const long long, const unsigned long)) {
    if (millis() - timePrev_ < kSampleTime_) {
        return;
    }
    
    input = func(resetVariable, millis() - timePrev_);

    // TODO: Call the method computeOutputModifier to replace the line 118 - 124
    const double error = setpoint - input;
    output = error * kP_ + errorSum_ * kI_ + (error - errorPrev_) * kD_;

    errorPrev_ = error;
    errorSum_ += error;
    errorSum_ = constrain(errorSum_, kMaxErrorSum_ * -1, kMaxErrorSum_);
    output = constrain(output, kMinOutput_, kMaxOutput_);
    resetVariable = 0;
    timePrev_ = millis();
}