#include "VLX.h"

VLX::VLX() {
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
            customPrintln("Comprueba que el VLX sirva");
            return false;
        }
        return true;
    }
    if (vLX_.begin()) {
        customPrintln("VLX OK");
        itWorks_ = true;
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
    double measure = (measure_.RangeMilliMeter / kMmInM);
    singleEMAFilter.addValue(measure);
    
    return singleEMAFilter.getLowPass();
}

void VLX::printDistance() {
    customPrint("Distance (M): ");
    customPrintln(VLX::getDistance());
} 