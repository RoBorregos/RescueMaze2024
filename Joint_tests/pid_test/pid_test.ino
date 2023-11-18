
#include "Motorclass.h"
//#include "Encoderclass.h"
unsigned long next_time; 
motor FRmotor(24,25,4);
float rpm = 0;

int tick1 =0;
float set_point = 80.00;
float error = 0.00;
float kp = 0.1;
float final_rpm = 0.00;
//496 ticks
void captureCallback(){
    //Serial.println(tick1);
    tick1++;
}
void get_rpm(){
    rpm = (tick1*60.00)/496.00;
    tick1 = 0;
    
}
void setup(){
    Serial.begin(115200);
    pinMode(18,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(18),captureCallback,RISING);
    next_time = millis();

}
void loop(){
    
    if(millis()-next_time>=1000){
        //Serial.print("1 second = ");
        //Serial.println(millis()-next_time);
        get_rpm();
        next_time = millis();
    }
    error = set_point - rpm;
    final_rpm = final_rpm-(kp*error);
    Serial.print("rpm: ");
    Serial.print(rpm);
    Serial.print(" error: ");
    Serial.print(error);
    Serial.print(" final_rpm: ");
    Serial.println(final_rpm);

    FRmotor.forward();
    FRmotor.set_pwm(80);


}