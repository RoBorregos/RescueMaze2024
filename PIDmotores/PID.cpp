#include "PID.h"

PID::PID() {
    timePrev = millis();
}

PID::PID(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime) {
    timePrev = millis();
    setTunings(kP, kI, kD);
    double sampleTime_ = sampleTime;
    errorPrev = 0;
    maxError = maxErrorSum;
    double minOutput_ = minOutput;
    double maxOutput_ = maxOutput;
}

PID::PID(const double kP, const double kI, const double kD) {
    timePrev = millis();
    setTunings(kP, kI, kD);
    errorPrev = 0;
}


void PID::computeStraight(const double targetOrientation, const double currentOrientation ,double &outputLeft, double &outputRight) {
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
    double outputModifier = kP * errorOrientation + kI * errorSum + kD * errorDeriv;
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
    

    Serial.println(outputLeft);
    Serial.println(outputRight);

    timePrev = millis(); 
}

void PID::computeStraightGiro(const double targetOrientation, const double currentOrientation ,double &outputLeft, double &outputRight) {
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
    double outputModifier = 20*(kP * errorOrientation + kI * errorSum + kD * errorDeriv) ;
    Serial.println("OUTPUTMODIFIER:" + String(outputModifier));
    int baseSpeed = 0; 
    if (errorOrientation <0){
        double outputModifieri = outputModifier * 2;
        outputLeft = outputModifieri;
        outputRight= -outputModifieri;
        Serial.println("Aumentando derecho");
        Serial.println("OUTPUTMODIFIER:" + String(outputModifier));
    }
    else if (errorOrientation >0){
        double outputModifierd = outputModifier * 2;
        outputRight = -outputModifierd;
        outputLeft= outputModifierd;
        Serial.println("Aumentando izquierdo");
        Serial.println("OUTPUTMODIFIER:" + String(outputModifier));

    }
    else{
        outputLeft = baseSpeed;
        outputRight = baseSpeed;
        Serial.println("Manteniendo");
    }
    outputLeft = constrain(outputLeft, -120, 120);
    outputRight = constrain(outputRight, -120, 120);

    errorPrev = errorOrientation;
    

    Serial.println(outputLeft);
    Serial.println(outputRight);

    timePrev = millis(); 
}

// NO COMENTAR
/* void PID::computeSpeedPerMotor(double targetSpeed, double &currentSpeed, uint8_t tics, double &output){
    unsigned long timeDiff = millis() - timePrev;
    unsigned long timeDiffInSec = timeDiff / 1000.0;
    currentSpeed = ((tics) / 496) * (timeDiffInSec);
    double errorSpeed = targetSpeed - currentSpeed;
    errorSum += errorSpeed * (timeDiffInSec);
    double errorDeriv = (errorSpeed - errorPrev) / (timeDiffInSec);
    output = kP * errorSpeed + kI * errorSum + kD * errorDeriv;
    errorPrev = errorSpeed;
    errorSum = max(maxError, min(-maxError, errorSum));
    output = max(minOutput, min(maxOutput, output));
    timePrev = millis();
    tics = 0;
} */

// no comentario
/* void PID::compute(const double setPoint, const double &input, double &output, int &resetVariable, const double PulsesPerRev, const double countTimeSampleInSec, const bool debug=false){
    unsigned long timeDiff = millis() - timePrev;
    if (timeDiff < sampleTime){
        return;
    }

    // Convert timeDiff to seconds
    const double timeDiffInSec = timeDiff / 1000.0;

    input =((resetVariable * PulsesPerRev) * (1000/timeDiff));

    resetVariable = 0;

    const double error = setPoint - input;

    errorSum += error * timeDiffInSec;

    const double errorDeriv = (error - errorPrev) / timeDiffInSec;

    output = kP * error + kI * errorSum + kD * errorDeriv;

    errorPrev = error;

    errorSum = max(maxError, min(-maxError, errorSum));
    output = max(minOutput, min(maxOutput, output));

    timePrev = millis();
    Serial.println("----[PID::compute]----");
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

    output = kP * error + kI * errorSum + kD * errorDeriv;

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

void PID::computeRotateLeft(const double target, const double current, double &output) {
    const unsigned long timeDiff = millis() - timePrev;

    if (timeDiff < sampleTime) {
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
    output = error * kP + errorSum * kI + (error - errorPrev) / timeDiff * kD;

    errorPrev = error;
    errorSum += error;

    errorSum = max(maxError, min(-maxError, errorSum));

    output = max(minOutput, min(maxOutput, output));

    timePrev = millis();
}

void PID::computeRotateRight(const double target, const double current, double &output) {
    const unsigned long timeDiff = millis() - timePrev;

    if (timeDiff < sampleTime) {
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
    output = error * kP + errorSum * kI + (error - errorPrev) / timeDiff * kD;

    errorPrev = error;
    errorSum += error;

    errorSum = max(maxError, min(-maxError, errorSum));

    output = max(minOutput, min(maxOutput, output));

    timePrev = millis();
}

//no comentar
void PID::setTunings(double kP, double kI, double kD) {
    this-> kP = kP;
    this-> kI = kI;
    this-> kD = kD;
}


 //no comentar
/* void reset(){
    errorSum = 0;
    errorPrev = 0;
} */

// no comentar
/* void infoPID(){
    Serial.println("PID info:");
    Serial.println("kP: " + String(kP));
    Serial.println("kI: " + String(kI));
    Serial.println("kD: " + String(kD));
    Serial.println("minOutput: " + String(minOutput));
    Serial.println("maxOutput: " + String(maxOutput));
    Serial.println("maxError: " + String(maxError));
    Serial.println("sampleTime: " + String(sampleTime));
} */