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
    public:
        BNO();
        void setupBNO();
        float getOrientationX();
};

#endif