#ifndef VLX_h
#define VLX_h

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include "CustomSerial.h"
#include "MUX.h"

#define VLX_ADDR 0x29

class VLX {
    private:
        Adafruit_VL53L0X vLX_ = Adafruit_VL53L0X();
        MUX mux_;
        VL53L0X_RangingMeasurementData_t measure_;

        const float kMm_in_M = 0.001;

    public:
        VLX();
        VLX(uint8_t posMux);
        void setMux(uint8_t posMux);
        double getDistance();
        float getRawDistance();
        void init();

        void printDistance();
};

#endif