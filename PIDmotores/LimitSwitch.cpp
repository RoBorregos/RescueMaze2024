#include "LimitSwitch.h"
#include "Pins.h"
#include "CustomSerial.h"

LimitSwitch::LimitSwitch() {
    state_ = false;
}

void LimitSwitch::initLimitSwitch(uint8_t pin) {
    pinMode(pin, INPUT);
}

bool LimitSwitch::getState(const LimitSwitchID id) {
    const uint8_t val = digitalRead(Pins::limitSwitchPins[static_cast<int>(id)]);
    state_ = (val == HIGH);
    return state_;
}