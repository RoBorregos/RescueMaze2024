#ifndef Encoderclass_h
#define Encoderclass_h

#include "Arduino.h"

class encoder{
    public:
        encoder(byte pinA, byte pinB);
        float get_rpm();

        
    private:
        byte pinA;
        byte pinB;
        static volatile int tick_A;
        static volatile int tick_B;
        static void callback_A();
        static void callback_B();
};

#endif