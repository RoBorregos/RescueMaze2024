#include "LimitSwitch.h"
#include "CustomSerial.h"

LimitSwitch::LimitSwitch() {
    this->state_ = false;
    this->pin_ = 0;
}

void LimitSwitch::initLimitSwitch(const uint8_t pin) {
    id_ = static_cast<LimitSwitchID>(pin);
    pin_ = pin;
    initLimitSwitchInternal();
}

void LimitSwitch::initLimitSwitchInternal() {
    pinMode(pin_, INPUT);
    customPrintln("LimitSwitch initialized");
    //attachInterrupt(digitalPinToInterrupt(pin_), LimitSwitchActive(), RISING);
}

bool LimitSwitch::getState() {
    const uint8_t val = digitalRead(pin_);
    if (val == HIGH) {
        customPrintln("LimitSwitch is active");
        state_ = true;
    } else {
        state_ = false;
    }
    return state_;
}

void LimitSwitch::LimitSwitchActive() {
    volatile bool val = digitalRead(pin_);
    if (state_ == HIGH) {
        customPrintln("LimitSwitch is active");
        state_ = true;
    } else {
        customPrintln("LimitSwitch is not active");
        state_ = false;
    }
}

void LimitSwitch::printState() {
    customPrint("LimitSwitch ");
    customPrint(static_cast<int>(id_));
    customPrint(" state: ");
    customPrintln(state_);
}
