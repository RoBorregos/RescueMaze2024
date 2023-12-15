#include "PID.h"

PID::PID(){
    tiempoPrev_=millis();
}

PID::PID(const double kp, const double ki, const double kd, const double errorMax, const double outputMax, const double outputMin, const long tiempoActual){
    
    
    tiempoPrev_=millis();
    setValues(kp, ki, kd);
    errorMax_=errorMax;
    outputMax_=outputMax;
    outputMin_=outputMin;
    tiempoActual_=tiempoActual;
}

void PID::setValues(double kp, double ki, double kd){
    kp_=kp;
    ki_=ki;
    kd_=kd;
}

void PID::controlSpeed(const double setpoint, double &input, double &output, int &reset, const double pulsosPorVuelta){
    if(millis()-tiempoPrev_<tiempoActual_){
        
        return;
    }


    input=(reset/pulsosPorVuelta)*(1000/(millis()-tiempoPrev_));
    reset=0;

    const double error=setpoint-input;

    errorAcum+=error*(millis()-tiempoPrev_);

    const double errorDeriv=(error-errorPrev)/millis()-tiempoPrev_;
    output=kp_*error+ki_*errorAcum+kd_*errorDeriv;


/*     if(output>outputMax){
        output=outputMax;
    }
    else if(output<outputMin){
        output=outputMin;
    } */

    errorPrev=error;
    errorAcum=errorAcum>errorMax_?errorMax_:errorAcum;
    output=output>outputMax_?outputMax_:output;
    output=output<outputMin_?outputMin_:output;
    tiempoPrev_=millis();

}
void PID::reset1(){
    errorAcum=0;
    errorPrev=0;
}
