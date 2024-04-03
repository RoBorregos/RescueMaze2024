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
        customPrintln("ERROR VLX");
        customPrint(mux_.hasAddress(VLX_ADDR));
        customPrintln(mux_.hasAddress(0x70));
        count++;
        if (count > kMaxInitAttempts_) {
            customPrint("Test the VLX: ");
            customPrintln(static_cast<uint8_t>(vlxId_));
            return false;
        }
    }
    customPrintln("VLX OK");
    itWorks_ = true;
    return true;
    
}

double VLX::getRawDistance() {
    updateDistance();
    const double measure = (static_cast<double>(measure_.RangeMilliMeter) / 1000.00);

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
    singleEMAFilter.addValue(measure);
    
    return singleEMAFilter.getLowPass();
}

void VLX::printDistance() {
    customPrint("Distance (M): ");
    customPrintln(VLX::getDistance());
} 