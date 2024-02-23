#include "VLX.h"


VLX::VLX() {
}

VLX::VLX(const uint8_t posMux) {
    mux_.setNewChannel(posMux);
}

void VLX::setMux(const uint8_t posMux) {
    mux_.setNewChannel(posMux);
}

void VLX::init() {
    mux_.selectChannel();
    while (!vLX_.begin()) {
        customPrintln("ERROR VLX");
        customPrint(mux_.hasAddress(VLX_ADDR));
        customPrintln(mux_.hasAddress(0x70));
    }
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
    double measure = (measure_.RangeMilliMeter / kMm_in_M);
    singleEMAFilter.AddValue(measure);
    
    return singleEMAFilter.GetLowPass();
}

void VLX::printDistance() {
    customPrint("Distance (M): ");
    customPrintln(VLX::getDistance());
}