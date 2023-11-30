/* 
#include <iostream>
#include "VLX.h"

using namespace std;

VLX::VLX(){
    mux = new MUX2C();
    mux->setAddress(VLX_ADDR);
    mux->begin();
    vlx->begin();
    vlx->setAddress(VLX_ADDR);
    vlx->setMeasurementTimingBudget(20000);
    vlx->startContinuous(20);
}

VLX::VLX(uint8_t posMux){
    mux = new MUX2C();
    mux->setAddress(VLX_ADDR);
    mux->begin();
    mux->setChannel(posMux);
    vlx->begin();
    vlx->setAddress(VLX_ADDR);
    vlx->setMeasurementTimingBudget(20000);
    vlx->startContinuous(20);
}

 */