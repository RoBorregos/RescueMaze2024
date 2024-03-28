#ifndef MUX_h
#define MUX_h

#include <Arduino.h>

#define MUX_ADDR 0x70

class MUX {
    private:
        uint8_t tcaPos_;

    public:
        MUX();
        
        void setNewChannel(const uint8_t tcaPos);
        void selectChannel(const uint8_t tcaPos);
        void selectChannel();
        void findI2C(const bool scan = true, const uint8_t address = 0);
        void setMatchingI2C(const uint8_t address);
        uint8_t getTcaPos();
        bool hasAddress(const uint8_t address);
};

#endif