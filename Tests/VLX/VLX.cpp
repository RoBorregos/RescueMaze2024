#include "VLX.h"

VLX::VLX() {
    this->vlxId_ = VlxID::kNone;
}

VLX::VLX(const uint8_t posMux) {
    mux_.setNewChannel(posMux);
}

void VLX::setMux(const uint8_t posMux) {
    mux_.setNewChannel(posMux);
}

bool VLX::init() {
    uint8_t count = 0;
    mux_.selectChannel();
    while (!vLX_.begin()) {
        Serial.println("ERROR VLX");
        Serial.print(mux_.hasAddress(VLX_ADDR));
        Serial.println(mux_.hasAddress(0x70));
        count++;
        if (count > kMaxInitAttempts_) {
            Serial.print("Test the VLX: ");
            Serial.println(static_cast<uint8_t>(vlxId_));
            return false;
        }
    }
    Serial.println("VLX OK");
    itWorks_ = true;
    return true;
    
}

uint16_t VLX::getRawDistance() {
    updateDistance();
    const double measure = (measure_.RangeMilliMeter / kMmInM_);

    return measure;
}

void VLX::updateDistance() {
    mux_.selectChannel();
    vLX_.rangingTest(&measure_, false);
}

double VLX::getDistance() {
    updateDistance();
    const double measure = (measure_.RangeMilliMeter / kMmInM_);
    //singleEMAFilter.addValue(measure);
    Serial.print(static_cast<uint8_t>(vlxId_));
    return measure; //singleEMAFilter.getLowPass();
}

void VLX::printDistance() {
    Serial.print("Distance (M): ");
    Serial.println(VLX::getDistance());
} 