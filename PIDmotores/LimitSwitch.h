#ifndef LimitSwitchh
#define LimitSwitch_h

#include "Arduino.h"

enum class LimitSwitchID {
    kLeft = 0,
    kRight = 1
};

class LimitSwitch {
    private:
        bool leftState_;
        bool rightState_;
        bool lastState_;
        bool state_;

    public:
        LimitSwitch();
        void initLimitSwitch();
        bool leftState();
        bool rightState();
        bool read(LimitSwitchID id);
        void getLimitSwitchState();
};

#endif