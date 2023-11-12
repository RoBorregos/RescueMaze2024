#ifndef PID_h
#define PID_h

#include <Arduino.h>
#include <math.h>

class PID{
    private:
        //Constantes
        double kp=0, ki=0, kd=0;
        //Variables externas
        double setpoint=0, input=0, output=0;
        //Variables internas
        double error=0, errorPrev=0, errorAcum=, errorDeriv=0;
    
        //Limites
        double errorMax=0;
        double outputMax=0 , outputMin=0;

        //Tiempo
        unsigned long  tiempoPrev=0, tiempoActual=0;
        
    public:
        PID(const double kp, const double ki, const double kd, const double errorMax, const double outputMax, const double outputMin, const long tiempoActual);
        PID();
        void setValues(double kp, double ki, double kd);
        void controlSpeed(double setpoint, double input);
};
#endif