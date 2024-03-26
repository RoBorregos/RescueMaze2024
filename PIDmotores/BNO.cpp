#include "BNO.h"
#include "CustomSerial.h"

BNO::BNO() {
    this->event = {0};
    this->bno = Adafruit_BNO055(55, 0x28, &Wire);
}
void BNO::setupBNO() {
    adafruit_bno055_opmode_t mode = OPERATION_MODE_IMUPLUS;

    customPrintln("Orientation Sensor Test");
    // Initialise the sensor
    if (!bno.begin()){
        // There was a problem detecting the BNO055 ... check your connections
        customPrint("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    delay(1000);
    bno.setExtCrystalUse(true);
}

void BNO::updateBNO(sensors_event_t &event) {
    bno.getEvent(&event);
}

double BNO::getOrientationX() {
    updateBNO(event);
    return event.orientation.x + phaseCorrection_;
}

double BNO::getOrientationY() {
    updateBNO(event);
    return event.orientation.y;
}

void BNO::setPhaseCorrection(const double phaseCorrection) {
    phaseCorrection_ = phaseCorrection;
}