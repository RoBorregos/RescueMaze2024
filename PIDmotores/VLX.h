#ifndef VLX_h
#define VLX_h

#include <Arduino.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_Sensor.h>
#include "MUX.h"
#include "CustomSerial.h"
#include "SingleEMAFilter.h"

#define VLX_ADDR 0x29


enum class VlxID {
    kFrontRight = 0,
    kBack = 1,
    kLeft = 2,
    kRight = 3,
    kFrontLeft = 4,
    kNone
};

class VLX {
    private:
        Adafruit_VL53L0X vLX_ = Adafruit_VL53L0X();
        MUX mux_;
        VL53L0X_RangingMeasurementData_t measure_;
        SingleEMAFilter<double> singleEMAFilter = SingleEMAFilter<double>(0.1);

        const double kMmInM = 1000.00;

        bool itWorks_ = false;

    public:
        VLX();
        VLX(const uint8_t posMux);
        void setMux(const uint8_t posMux);
        double getDistance();
        uint16_t getRawDistance();
        bool init();
        void updateDistance();

        void printDistance();
};

#endif