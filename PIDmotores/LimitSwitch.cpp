#include "LimitSwitch.h"
#include "CustomSerial.h"

LimitSwitch::LimitSwitch() {
    state_ = false;
    this->pin_ = 0;
}

void LimitSwitch::initLimitSwitch(const uint8_t pin) {
    id_ = static_cast<LimitSwitchID>(pin);
    pin_ = pin;
    pinMode(pin_, INPUT);
}

bool LimitSwitch::getState() {
    const uint8_t val = digitalRead(pin_);
    state_ = (val == HIGH);
    return state_;
}

void LimitSwitch::printState() {
    Serial.print("LimitSwitch ");
    Serial.print(static_cast<int>(id_));
    Serial.print(" state: ");
    Serial.println(state_);
}
```