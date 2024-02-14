#include "MUX.h"

MUX::MUX() {
    tcaPos_ = 0;
}

void MUX::setNewChannel(uint8_t tcaPos) {
    this->tcaPos_ = tcaPos;
}

void MUX::selectChannel(uint8_t tcaPos) {
    setNewChannel(tcaPos);
    selectChannel();
}

void MUX::selectChannel() {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1 << tcaPos_);
    Wire.endTransmission();
}

void MUX::findI2C(bool scan, uint8_t address) {
    if (scan) {
        Wire.begin();
        for (uint8_t i = 0; i < 128; i++) {
            Wire.beginTransmission(i);
            if (Wire.endTransmission() == 0) {
                Serial.print("Found address: ");
                Serial.println(i, DEC);
            }
        }
    } else {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found address: ");
            Serial.println(address, DEC);
        }
    }
}

void MUX::setMatchingI2C(uint8_t address) {
    findI2C(false, address);
}

uint8_t MUX::getTcaPos() {
    return tcaPos_;
}