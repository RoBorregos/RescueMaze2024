#ifndef Encoderclass_h
#define Encoderclass_h

#include "Arduino.h"

class encoder{
    public:
        encoder(byte pinA);
        float get_rpm();

        
    private:
        byte pinA;
        static volatile int tick_A;
        static void callback_A();
        static void callback_B();
};

#endif