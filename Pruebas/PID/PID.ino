#include <Arduino.h>




//Constantes
double Kp=0, Ki=0, Kd=0;

//Variables externas
double setpoint=0, input=0, output=0;

//Variables internas
double error=0, errorPrev=0, errorAcum=, errorDeriv=0;
unsigned long tiempoPrev=0, tiempoActual=0;
double tiempoTranscurrido=0;



void setup(){


}
void loop(){

}


