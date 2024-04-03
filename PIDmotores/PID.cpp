#include "PID.h"
#include "CustomSerial.h"

#define DEBUG_PID 0

PID::PID() {
    timePrev_ = millis();
    errorPrev_ = 0;

    #if DEBUG_PID
    customPrintln("kP_1:" + String(kP_));
    #endif

}

PID::PID(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime, double baseModifier, double kMaxError) {
    timePrev_ = millis();
    errorPrev_ = 0;
    kBaseModifier_ = baseModifier;
    kMaxError_ = kMaxError;
    setTunnings(kP, kI, kD, minOutput, maxOutput, maxErrorSum, sampleTime, baseModifier, kMaxError);
    #if DEBUG_PID
    customPrintln("kP9:" + String(kP));
    #endif
}

PID::PID(const double kP, const double kI, const double kD) {
    timePrev_ = millis();
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;

    errorPrev_ = 0;
    #if DEBUG_PID
    customPrintln("kP3:" + String(kP));
    #endif
}

void PID::setBaseSpeed(const double baseModifier) {
    kBaseModifier_ = baseModifier;
}

void PID::setTunnings(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime, const double baseModifier = 0, const double maxError = 0) {
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
    kMinOutput_ = minOutput;
    kMaxOutput_ = maxOutput;
    kMaxErrorSum_ = maxErrorSum;
    kSampleTime_ = sampleTime;
    kBaseModifier_ = baseModifier;
    kMaxError_ = maxError;
    #if DEBUG_PID
    customPrintln("kP:" + String(kP));
    customPrintln("kP_" + String(kP_));
    #endif
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
    errorSum_ += error;
    errorSum_ = constrain(errorSum_, kMaxErrorSum_ * -1, kMaxErrorSum_);
    const double errorDeriv = (error - errorPrev_) / (timeDiff);
    const double outputModifier = kP_ * error + kI_ * errorSum_ + kD_ * errorDeriv;
    errorPrev_ = error;
    if (timeDiff < kSampleTime_) {
        #if DEBUG_PID
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
        #endif
    }
    return outputModifier;
}

double PID::computeOutputModifierVlx(const double error, const unsigned long timeDiff) {
    errorSumVlx_ += error;
    errorSumVlx_ = constrain(errorSumVlx_, kMaxErrorSum_ * -1, kMaxErrorSum_);
    const double errorDeriv = (error - errorPrevVlx_) / (timeDiff);
    const double outputModifier = kP_ * error + kI_ * errorSumVlx_ + kD_ * errorDeriv;
    errorPrevVlx_ = error;
    if (timeDiff < kSampleTime_) {
        #if DEBUG_PID
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
        #endif
    }
    return outputModifier;

}

void PID::computeStraight(const double targetOrientation, const double currentOrientation ,double &outputLeft, double &outputRight) {
    const unsigned long timeDiff = millis() - timePrev_;
    if (timeDiff < kSampleTime_) {
        #if DEBUG_PID
        customPrintln("TimeDiff:" + String(timeDiff));
        #endif
        return;
    }
    const double errorOrientation = computeErrorOrientation(targetOrientation, currentOrientation);
    const double outputModifier = computeOutputModifier(errorOrientation, timeDiff);
    
    #if DEBUG_PID
    customPrintln("ERRORORIENTATION:" + String(errorOrientation));
    customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
    customPrintln("KP" + String(this->kP_));
    customPrintln("KI" + String(kI_));
    customPrintln("KD" + String(kD_));
    customPrintln("kMinOutput_" + String(kMinOutput_));
    customPrintln("kMaxOutput_" + String(kMaxOutput_));
    customPrintln("kMaxErrorSum_" + String(kMaxErrorSum_));
    customPrintln("kSampleTime_" + String(kSampleTime_));
    customPrintln("kBaseModifier_" + String(kBaseModifier_));
    customPrintln("kMaxOrientationError_" + String(kMaxOrientationError_));
    #endif

    outputLeft = kBaseModifier_;
    outputRight = kBaseModifier_;
    if (abs(errorOrientation) > kMaxError_) {
        outputLeft += outputModifier;
        outputRight -= outputModifier;
    }
    
    #if DEBUG_PID
    customPrintln("outputLeft" + String(outputLeft));
    customPrintln("outputRight" + String(outputRight));
    customPrintln("baseModifier" + String(kBaseModifier_));
    #endif
    
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

    if (errorOrientation < kMaxError_) {
        speed = kBaseModifier_ + outputModifier;
        clockwise = true;
        #if DEBUG_PID
        customPrintln("Aumentando derecho");
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
        #endif
    } else if (errorOrientation > -kMaxError_) {
        speed = kBaseModifier_ - outputModifier;
        clockwise = false;
        #if DEBUG_PID
        customPrintln("Aumentando izquierdo");
        customPrintln("OUTPUTMODIFIER:" + String(outputModifier));
        #endif
    } else { 
        speed = 0;
        goalReached = true;
    }
    
    if (goalReached == false) {
        speed = constrain(speed, kMinOutput_, kMaxOutput_);
    }
    #if DEBUG_PID
    customPrintln("SPEED:" + String(speed));
    #endif
    errorPrev_ = errorOrientation;

    timePrev_ = millis();
}

void PID::compute(const double setpoint, double& input, double& output, long long &resetVariable, double (*func)(const long long, const unsigned long)) {
    const unsigned long timeDiff = millis() - timePrev_;
    if (timeDiff < kSampleTime_) {
        return;
    }
    
    input = func(resetVariable, timeDiff);

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

void PID::computeDistance(const double setpoint, const double input, double& outputLeft, double& outputRight) {
    const unsigned long timeDiff = millis() - timePrev_;
    if (timeDiff < kSampleTime_) {
        return;
    }

    const double error = setpoint - input;
    const double outputModifier = computeOutputModifier(error, timeDiff);

    outputLeft = kBaseModifier_;
    outputRight = kBaseModifier_;
    
    if (abs(error) > kMaxError_) {
        outputLeft += outputModifier;
        outputRight -= outputModifier;
    }

    outputLeft = constrain(outputLeft, kMinOutput_, kMaxOutput_);
    outputRight = constrain(outputRight, kMinOutput_, kMaxOutput_);
    errorPrev_ = error;

    timePrev_ = millis();
}

void PID::computeWeightedPID(const double targetDistance, const double currentDistance, const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight) {
    const unsigned long timeDiff = millis() - timePrev_;
    if (timeDiff < kSampleTime_) {
        return;
    }

    const double error = targetDistance - currentDistance;
    const double errorOrientation = computeErrorOrientation(targetOrientation, currentOrientation);
    const double outputModifier = computeOutputModifierVlx(error, timeDiff);
    const double outputModifierOrientation = computeOutputModifier(errorOrientation, timeDiff);

    const double weightPID = outputModifier * kWeightVlx + outputModifierOrientation * kWeightBNO;

    outputLeft = kBaseModifier_;
    outputRight = kBaseModifier_;

    if (abs(error) > kMaxError_) {
        outputLeft += weightPID;
        outputRight -= weightPID;
    }

    if (abs(errorOrientation) > kMaxError_) {
        outputLeft += weightPID;
        outputRight -= weightPID;
    }

    outputLeft = constrain(outputLeft, kMinOutput_, kMaxOutput_);
    outputRight = constrain(outputRight, kMinOutput_, kMaxOutput_);

    errorPrev_ = error;

    timePrev_ = millis();
}