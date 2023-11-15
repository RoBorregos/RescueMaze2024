#ifndef PID_h
#define PID_h

#include <Arduino.h>
#include <math.h>

class PID{
    private:
        //Constantes
        double kp_=0, ki_=0, kd_=0;
        //Variables externas
        double setpoint=0, input=0, output=0;
        //Variables internas
        double error=0, errorPrev=0, errorAcum=0, errorDeriv=0;
    
        //Limites
        double errorMax_=0;
        double outputMax_=0 , outputMin_=0;

        //Tiempo
        unsigned long  tiempoPrev_=0, tiempoActual_=0;
        
    public:
        PID(const double kp, const double ki, const double kd, const double errorMax, const double outputMax, const double outputMin, const long tiempoActual);
        PID();
        void setValues(double kp, double ki, double kd);
        void controlSpeed(const double setpoint, double &input, double &output, int &reset, const double pulsosPorVuelta);
};
#endif