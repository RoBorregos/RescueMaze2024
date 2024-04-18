//pines de motores
#include <Arduino.h>
namespace Pins {
    constexpr int pwmPin[4] = {
        15, // BACK_LEFT 16   5
        26, // FRONT_LEFT 32
        5, // BACK_RIGHT 15   15
        32  // FRONT_RIGHT 26
    };

    constexpr int digitalOne[4] = {
        4, // BACK_LEFT 17   17
        14, // FRONT_LEFT 33
        16, // BACK_RIGHT 4    2
        33  // FRONT_RIGHT 14
    };  

    constexpr int digitalTwo[4] = {
        2, // BACK_LEFT 5  16
        27, // FRONT_LEFT 25 
        17, // BACK_RIGHT 2   4
        25  // FRONT_RIGHT 27
    };

    constexpr int encoderA[4] = {
        34, // BACK_LEFT 35    35
        36, // FRONT_LEFT 39
        35, // BACK_RIGHT 34   34
        39  // FRONT_RIGHT 36
    };

    constexpr int vlxPins[5] = {
        // Defined Pins on 20th of March
        2,  // FRONT_LEFT  fine 0
        3, // LEFT         fine 1        
        0, // BACK         fine 2
        5,  // RIGHT       fine 4
        4 // FRONT_RIGHT   fine 3
    };

    constexpr int limitSwitchPins[2] = {
        13, // LEFT
        23  // RIGHT
    };

    constexpr int tcsPins[1] = {
        1 // 5
    };

    constexpr int servoPin = 18;

    constexpr int buttonPin = 19;
}

int pwm = 255;

void avanzar(){
    // analogWrite(Pins::pwmPin[2],pwm);
    // digitalWrite(Pins::digitalOne[2],HIGH);
    // digitalWrite(Pins::digitalTwo[2],LOW);
    analogWrite(Pins::pwmPin[0],pwm);
    digitalWrite(Pins::digitalOne[0],HIGH);
    digitalWrite(Pins::digitalTwo[0],LOW);
    // analogWrite(Pins::pwmPin[3],pwm);
    // digitalWrite(Pins::digitalOne[3],HIGH);
    // digitalWrite(Pins::digitalTwo[3],LOW);
    // analogWrite(Pins::pwmPin[1],pwm);
    // digitalWrite(Pins::digitalOne[1],HIGH);
    // digitalWrite(Pins::digitalTwo[1],LOW);
}

void girarDer(){
    // analogWrite(Pins::pwmPin[2],pwm);
    // digitalWrite(Pins::digitalOne[2],HIGH);
    // digitalWrite(Pins::digitalTwo[2],LOW);
    analogWrite(Pins::pwmPin[0],pwm);
    digitalWrite(Pins::digitalOne[0],LOW);
    digitalWrite(Pins::digitalTwo[0],HIGH);
    // analogWrite(Pins::pwmPin[3],pwm);
    // digitalWrite(Pins::digitalOne[3],HIGH);
    // digitalWrite(Pins::digitalTwo[3],LOW);
    // analogWrite(Pins::pwmPin[1],pwm);
    // digitalWrite(Pins::digitalOne[1],LOW);
    // digitalWrite(Pins::digitalTwo[1],HIGH);
    
}
void girarIzq(){
    // analogWrite(Pins::pwmPin[2],pwm);
    // digitalWrite(Pins::digitalOne[2],LOW);
    // digitalWrite(Pins::digitalTwo[2],HIGH);
    analogWrite(Pins::pwmPin[0],pwm);
    digitalWrite(Pins::digitalOne[0],HIGH);
    digitalWrite(Pins::digitalTwo[0],LOW);
    // analogWrite(Pins::pwmPin[3],pwm);
    // digitalWrite(Pins::digitalOne[3],LOW);
    // digitalWrite(Pins::digitalTwo[3],HIGH);
    // analogWrite(Pins::pwmPin[1],pwm);
    // digitalWrite(Pins::digitalOne[1],HIGH);
    // digitalWrite(Pins::digitalTwo[1],LOW);
}
void atras(){
    // analogWrite(Pins::pwmPin[2],pwm);
    // digitalWrite(Pins::digitalOne[2],LOW);
    // digitalWrite(Pins::digitalTwo[2],HIGH);
    analogWrite(Pins::pwmPin[0],pwm);
    digitalWrite(Pins::digitalOne[0],LOW);
    digitalWrite(Pins::digitalTwo[0],HIGH);
    // analogWrite(Pins::pwmPin[3],pwm);
    // digitalWrite(Pins::digitalOne[3],LOW);
    // digitalWrite(Pins::digitalTwo[3],HIGH);
    // analogWrite(Pins::pwmPin[1],pwm);
    // digitalWrite(Pins::digitalOne[1],LOW);
    // digitalWrite(Pins::digitalTwo[1],HIGH);
}
void parar(){
    // analogWrite(Pins::pwmPin[2],0);
    // digitalWrite(Pins::digitalOne[2],LOW);
    // digitalWrite(Pins::digitalTwo[2],LOW);
    analogWrite(Pins::pwmPin[0],0);
    digitalWrite(Pins::digitalOne[0],LOW);
    digitalWrite(Pins::digitalTwo[0],LOW);
    // analogWrite(Pins::pwmPin[3],0);
    // digitalWrite(Pins::digitalOne[3],LOW);
    // digitalWrite(Pins::digitalTwo[3],LOW);
    // analogWrite(Pins::pwmPin[1],0);
    // digitalWrite(Pins::digitalOne[1],LOW);
    // digitalWrite(Pins::digitalTwo[1],LOW);
}
void setup (){
    Serial.begin(115200);
    for(int i = 0; i < 4; i++){
        pinMode(Pins::pwmPin[i],OUTPUT);
        pinMode(Pins::digitalOne[i],OUTPUT);
        pinMode(Pins::digitalTwo[i],OUTPUT);
    }
}
void loop(){
    // ++pwm;
    for(int i = 0; i < 4; i++){
      analogWrite(Pins::pwmPin[i],pwm);
      digitalWrite(Pins::digitalOne[i],LOW);
      digitalWrite(Pins::digitalTwo[i],HIGH);
      delay(1000);  
    }
}