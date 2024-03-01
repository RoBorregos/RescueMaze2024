#ifndef LimitSwitchh
#define LimitSwitch_h

#include "Arduino.h"

enum class LimitSwitchID {
    kLeft = 0,
    kRight = 1
};

class LimitSwitch {
    private:
        bool state_;
        LimitSwitchID id_;

    public:
        LimitSwitch();
        void initLimitSwitch(uint8_t pin);
        bool getState(const LimitSwitchID id);
};

#endif