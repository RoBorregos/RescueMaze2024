#include <Arduino.h>



//encoder 2, 3 19 18
//enas 4,5, 6, 7

//Constantes
double Kp=0, Ki=0, Kd=0;

//Variables externas
double setpoint=0, input=0, output=0;

//Variables internas
double error=0, errorPrev=0, errorAcum=0, errorDeriv=0;
unsigned long tiempoPrev=0, tiempoActual=0;
double tiempoTranscurrido=0;


/* int ENA= 7;
int IN1= 26;
int IN2= 27;
int IN3= 28;
int IN4= 29;
int ENB= 6; */

void setup(){
    //motores
    //motorSetup();
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);


}
void loop(){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}


