#include "VLX.h"

VLX::VLX() {
}

VLX::VLX(uint8_t posMux) {
    mux_.setNewChannel(posMux);
}

void VLX::setMux(uint8_t posMux) {
    mux_.setNewChannel(posMux);
}

void VLX::init() {
    mux_.selectChannel();
    if (!vLX_.begin()){
        customPrintln("ERROR VLX");
    }
}

float VLX::getRawDistance() {
    mux_.selectChannel();
    vLX_.rangingTest(&measure_, false);

    return measure_.RangeMilliMeter;
}

double VLX::getDistance() {
    mux_.selectChannel();
    vLX_.rangingTest(&measure_, false);

    return (measure_.RangeMilliMeter / 1000.00);
}

void VLX::printDistance() {
    customPrint("Distance (M): ");
    customPrintln(VLX::getDistance());
}