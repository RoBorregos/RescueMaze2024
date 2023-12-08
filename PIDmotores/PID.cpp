#include "PID.h"

PID::PID(){
    timePrev = millis();
}

PID::PID(const double kp, const double ki, const double kd, const double minOutput_, const double maxOutput_, const double maxErrorSum, const long sampleTime_){
    timePrev = millis();
    setTunings(kp, ki, kd);
    sampleTime = sampleTime_;
    errorPrev = 0;
    maxError = maxErrorSum;
    minOutput = minOutput_;
    maxOutput = maxOutput_;
}

PID::PID(const double kp, const double ki, const double kd){
    timePrev = millis();
    setTunings(kp, ki, kd);
    errorPrev = 0;
}


void PID::computeStraight(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight){
    unsigned long timeDiff = millis() - timePrev;
    
    double errorOrientation = targetOrientation - currentOrientation;
    if (errorOrientation > 180) {
        errorOrientation -= 360;
    }
    else if (errorOrientation < -180) {
        errorOrientation += 360;
    }   

    errorSum += errorOrientation * (timeDiff);
    double errorDeriv = (errorOrientation - errorPrev) / (timeDiff);
    double outputModifier = kp * errorOrientation + ki * errorSum + kd * errorDeriv;
    int baseSpeed = 70;
    if (errorOrientation <0){
        outputLeft = baseSpeed+outputModifier;
        outputRight= baseSpeed-outputModifier;
        Serial.println("Aumentando derecho");
        Serial.println("OUTPUTMODIFIER:" + String(outputModifier));
    }
    else if (errorOrientation >0){
        outputRight = baseSpeed-outputModifier;
        outputLeft= baseSpeed+outputModifier;
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

    errorPrev = errorOrientation;
    

    timePrev = millis(); 
}

// no comentario
/* void PID::compute(const double setPoint, const double &input, double &output, int &resetVariable, const double PulsesPerRev, const double countTimeSampleInSec, const bool debug=false){
    unsigned long timeDiff = millis() - timePrev;
    if (timeDiff < sampleTime){
        return;
    }

    // Convert timeDiff to seconds
    double timeDiffInSec = timeDiff / 1000.0;

    input =((resetVariable * PulsesPerRev) * (1000/timeDiff));

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
} */
// TODO: HACER FUNCION PARA QUE CONSIDERE EL BNO PARA QUE SE VAYA DERECHO

// no comentar
/* void PID::compute(const double setPoint, double &input, double &output, int &resetVariable, const double PulsesPerRev, const double countTimeSampleInSec, const bool debug=false){
    unsigned long timeDiff = millis() - timePrev;
    if (timeDiff < sampleTime){
        return;
    }

    // Convert timeDiff to seconds
    double timeDiffInSec = timeDiff / 1000.0;

    input = ((resetVariable * PulsesPerRev) * (1000/timeDiff));

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
} */
// .............................................................................

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

//no comentar
void PID::setTunings(double kp, double ki, double kd){
    this-> kp = kp;
    this-> ki = ki;
    this-> kd = kd;
}


 //no comentar
/* void reset(){
    errorSum = 0;
    errorPrev = 0;
} */

// no comentar
/* void infoPID(){
    Serial.println("PID info:");
    Serial.println("kp: " + String(kp));
    Serial.println("ki: " + String(ki));
    Serial.println("kd: " + String(kd));
    Serial.println("minOutput: " + String(minOutput));
    Serial.println("maxOutput: " + String(maxOutput));
    Serial.println("maxError: " + String(maxError));
    Serial.println("sampleTime: " + String(sampleTime));
} */