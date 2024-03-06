#include "LimitSwitch.h"
#include "CustomSerial.h"

#define DEBUG_LIMIT_SWITCH 0

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
    #if DEBUG_LIMIT_SWITCH
    customPrintln("LimitSwitch initialized");
    #endif
}

bool LimitSwitch::getState() {
    const uint8_t val = digitalRead(pin_);
    if (val == HIGH) {
        #if DEBUG_LIMIT_SWITCH
        customPrintln("LimitSwitch is active");
        #endif
        state_ = true;
    } else {
        state_ = false;
    }
    return state_;
}

void LimitSwitch::LimitSwitchActive() {
    volatile bool val = digitalRead(pin_);
    if (state_ == HIGH) {
        #if DEBUG_LIMIT_SWITCH
        customPrintln("LimitSwitch is active");
        #endif
        state_ = true;
    } else {
        #if DEBUG_LIMIT_SWITCH
        customPrintln("LimitSwitch is not active");
        #endif
        state_ = false;
    }
}

void LimitSwitch::printState() {
    #if DEBUG_LIMIT_SWITCH
    customPrint("LimitSwitch ");
    customPrint(static_cast<int>(id_));
    customPrint(" state: ");
    customPrintln(state_);
    #endif
}
