#include "LimitSwitch.h"
#include "Pins.h"

LimitSwitch::LimitSwitch() {
    state_ = false;
    lastState_ = false;
}

void LimitSwitch::initLimitSwitch() {
    pinMode(Pins::limitSwitchPins[static_cast<int>(LimitSwitchID::kLeft)], INPUT);
    pinMode(Pins::limitSwitchPins[static_cast<int>(LimitSwitchID::kRight)], INPUT);
}

bool LimitSwitch::leftState() {
    int val = digitalRead(Pins::limitSwitchPins[static_cast<int>(LimitSwitchID::kLeft)]);
    leftState_ = (val == HIGH);
    return leftState_;
}

bool LimitSwitch::rightState() {
    int val = digitalRead(Pins::limitSwitchPins[static_cast<int>(LimitSwitchID::kRight)]);
    rightState_ = (val == HIGH);
    return rightState_;
}

bool LimitSwitch::read(LimitSwitchID id) {
    state_ = digitalRead(Pins::limitSwitchPins[static_cast<int>(id)]);
    return state_;
}

void LimitSwitch::getLimitSwitchState() {
    leftState();
    rightState();
} 
