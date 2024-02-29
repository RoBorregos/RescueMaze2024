#include "LimitSwitch.h"
#include "Pins.h"
#include "CustomSerial.h"

LimitSwitch::LimitSwitch() {
    state_ = false;
}

void LimitSwitch::initLimitSwitch() {
    pinMode(Pins::limitSwitchPins[static_cast<uint8_t>(LimitSwitchID::kLeft)], INPUT);
    pinMode(Pins::limitSwitchPins[static_cast<uint8_t>(LimitSwitchID::kRight)], INPUT);
}

bool LimitSwitch::getState(LimitSwitchID id) {
    int val = digitalRead(Pins::limitSwitchPins[static_cast<int>(id)]);
    state_ = (val == HIGH);
    return state_;
}

void LimitSwitch::printLimitSwitchState() {
    customPrint("Left limit switch: ");
    customPrintln(getState(LimitSwitchID::kLeft));
    customPrint("Right limit switch: ");
    customPrintln(getState(LimitSwitchID::kRight));
}
