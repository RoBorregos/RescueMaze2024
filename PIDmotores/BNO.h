#ifndef BNO_h
#define BNO_h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class BNO{
    private:
        Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
        float orientationX;
        sensors_event_t event;
    public:
        BNO();
        void setupBNO();
        void updateBNO(sensors_event_t &event);
        float getOrientationX();
};

#endif