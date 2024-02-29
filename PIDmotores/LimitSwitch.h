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

    public:
        LimitSwitch();
        void initLimitSwitch();
        bool getState(LimitSwitchID id);
        void printLimitSwitchState();
};

#endif