#include "PID.h"

PID::PID(){
    tiempoPrev=millis();
}

PID::PID(const double kp, const double ki, const double kd, const double errorMax, const double outputMax, const double outputMin, const long tiempoActual){
    
    
    tiempoPrev=millis();
    setValues(kp, ki, kd);
    this->errorMax=errorMax;
    this->outputMax=outputMax;
    this->outputMin=outputMin;
    this->tiempoActual=tiempoActual;
}

void setValues(double kp, double ki, double kd){
    this->kp=kp;
    this->ki=ki;
    this->kd=kd;
}

void PID::controlSpeed(const double set){
    if(millis()-tiempoPrev<tiempoActual){
        tiempoPrev=millis();
        return;
    }
    //aqui iria el input 

    const double error=setpoint-input;
    const double errorAcum=errorAcum+error;
    const double errorDeriv=error-errorPrev;
    const double output=kp*error+ki*errorAcum+kd*errorDeriv;

    if(output>outputMax){
        output=outputMax;
    }
    else if(output<outputMin){
        output=outputMin;
    }

    errorPrev=error;
    errorAcum+=error;

    errorAcum=errorAcum>errorMax?errorMax:errorAcum;
    output=output>outputMax?outputMax:output;
    output=output<outputMin?outputMin:output;

}
