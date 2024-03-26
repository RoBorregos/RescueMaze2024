#ifndef BNO_h
#define BNO_h

#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class BNO {
    private:
        Adafruit_BNO055 bno;
        sensors_event_t event;
        double phaseCorrection_ = 0.0;
    public:
        // TODO: AGREGAR MAS COMENTARIOS SOBRE FUNCIONAMIENTO
        BNO();
        void setupBNO();
        void updateBNO(sensors_event_t &event);
        double getOrientationX();
        double getOrientationY();
        void setPhaseCorrection(const double phaseCorrection);
};

#endif