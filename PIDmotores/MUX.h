#ifndef MUX_h
#define MUX_h

#include <Arduino.h>
#include <Wire.h>

#define MUX_ADDR 0x70

class MUX {
    private:
        uint8_t tcaPos_;

    public:
        MUX();
        
        void setNewChannel(uint8_t tcaPos);
        void selectChannel(uint8_t tcaPos);
        void selectChannel();
        void findI2C(bool scan, uint8_t address);
        void setMatchingI2C(uint8_t address);
        uint8_t getTcaPos();
};

#endif