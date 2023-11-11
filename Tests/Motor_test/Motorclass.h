#ifndef Motorclass_h
#define Motorclass_h

#include "Arduino.h"

class motor{
    public:
        motor(byte pin1, byte pin2, byte pwm_pin);
        void forward();
        void backward();
        void stop();
        void set_pwm(byte pwm);
    private:
        byte pin1;
        byte pin2;
       byte pwm_pin;
};

#endif