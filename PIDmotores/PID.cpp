#include "PID.h"

PID::PID(){
    timePrev = millis();
}

PID::PID(const double kp, const double ki, const double kd, const double minOutput_, const double maxOutput_, const double maxErrorSum, const long sampleTime_){
    timePrev = millis();
    setTunings(kp, ki, kd);
    sampleTime = sampleTime_;

    maxError = maxErrorSum;
    minOutput = minOutput_;
    maxOutput = maxOutput_;
}

PID::PID(const double kp, const double ki, const double kd){
    timePrev = millis();
    setTunings(kp, ki, kd);
}

void PID::compute(const double setPoint, const double &input, double &output, int &resetVariable, const double PulsesPerRev, const double countTimeSampleInSec, const bool debug=false){
    unsigned long timeDiff = millis() - timePrev;
    if (timeDiff < sampleTime){
        return;
    }

    // Convert timeDiff to seconds
    double timeDiffInSec = timeDiff / 1000.0;

    input =(resetVariable * PulsesPerRev) * (1000/timeDiff);

    resetVariable = 0;

    const double error = setPoint - input;

    errorSum += error * timeDiffInSec;

    const double errorDeriv = (error - errorPrev) / timeDiffInSec;

    output = kp * error + ki * errorSum + kd * errorDeriv;

    errorPrev = error;

    errorSum = max(maxError, min(-maxError, errorSum));
    output = max(minOutput, min(maxOutput, output));

    timePrev = millis();

    if (debug) {
        Serial.println("Time diff:" + String(timeDiff));
        Serial.println("Input:" + String(input));
        Serial.println("Error:" + String(error));
        Serial.println("Error sum:" + String(errorPrev));
        Serial.println("Error deriv:" + String(errorDeriv));
        Serial.println("Error sum:" + String(errorSum));
        Serial.println("Output:" + String(output));
    }
}

void PID::computeRotateIzq(const double target, const double current, double &output){
    unsigned long timeDiff = millis() - timePrev;

    if (timeDiff < sampleTime){
        return;
    }

    double error = 0;

    if (current < target) {

        error = 360 - target + current;
    }
    else if (target == 0) {
        error = current;
    }
    else {
        error = current - target;
    }

    // Warning: Check if the units of the time are correct (milliseconds)
    output = error * kp + errorSum * ki + (error - errorPrev) / timeDiff * kd;

    errorPrev = error;
    errorSum += error;

    errorSum = max(maxError, min(-maxError, errorSum));

    output = max(minOutput, min(maxOutput, output));

    timePrev = millis();
}

void PID::computeRotateDer(const double target, const double current, double &output){
    unsigned long timeDiff = millis() - timePrev;

    if (timeDiff < sampleTime){
        return;
    }

    double error = 0;

    if (current > target) {

        error = 360 - current + target;
    }
    else if (target == 0) {
        error = 360 - current;
    }
    else {
        error = target - current;
    }

    // Warning: Check if the units of the time are correct (milliseconds)
    output = error * kp + errorSum * ki + (error - errorPrev) / timeDiff * kd;

    errorPrev = error;
    errorSum += error;

    errorSum = max(maxError, min(-maxError, errorSum));

    output = max(minOutput, min(maxOutput, output));

    timePrev = millis();
}

void setTunings(double kp, double ki, double kd){
    this-> kp = kp;
    this-> ki = ki;
    this-> kd = kd;
}

void reset(){
    errorSum = 0;
    errorPrev = 0;
}

void infoPID(){
    Serial.println("PID info:");
    Serial.println("kp: " + String(kp));
    Serial.println("ki: " + String(ki));
    Serial.println("kd: " + String(kd));
    Serial.println("minOutput: " + String(minOutput));
    Serial.println("maxOutput: " + String(maxOutput));
    Serial.println("maxError: " + String(maxError));
    Serial.println("sampleTime: " + String(sampleTime));
}