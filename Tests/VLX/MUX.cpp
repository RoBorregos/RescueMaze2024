#include "MUX.h"
#include <Wire.h>


MUX::MUX() {
    tcaPos_ = 0;
}

void MUX::setNewChannel(const uint8_t tcaPos) {
    this->tcaPos_ = tcaPos;
}

void MUX::selectChannel(const uint8_t tcaPos) {
    setNewChannel(tcaPos);
    selectChannel();
}

void MUX::selectChannel() {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1 << tcaPos_);
    Wire.endTransmission();
}

void MUX::findI2C(const bool scan, const uint8_t address) {
    if (scan) {
        for (uint8_t i = 0; i < 128; i++) {
            Wire.beginTransmission(i);
            if (Wire.endTransmission() == 0) {
                Serial.print("Found address: ");
                Serial.println(i);
            }
        }
    } else {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found address: ");
            Serial.println(address);
        }
    }
}

void MUX::setMatchingI2C(const uint8_t address) {
    findI2C(false, address);
}

uint8_t MUX::getTcaPos() {
    return tcaPos_;
}

bool MUX::hasAddress(const uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}