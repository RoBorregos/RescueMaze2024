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
        if (count > 1000) {
            customPrintln("Test the VLX" + String(vlxId_));
            return false;
        }
    }
        customPrintln("VLX OK");
        itWorks_ = true;
        return true;
    
}

uint16_t VLX::getRawDistance() {
    updateDistance();

    return measure_.RangeMilliMeter;
}

void VLX::updateDistance() {
    mux_.selectChannel();
    vLX_.rangingTest(&measure_, false);
}

double VLX::getDistance() {
    updateDistance();
    const double measure = (measure_.RangeMilliMeter / kMmInM);
    singleEMAFilter.addValue(measure);
    
    return singleEMAFilter.getLowPass();
}

void VLX::printDistance() {
    customPrint("Distance (M): ");
    customPrintln(VLX::getDistance());
} 