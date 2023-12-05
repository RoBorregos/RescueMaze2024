#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "BNO.h"

BNO::BNO(){
    this-> event = {0};
    this-> bno = Adafruit_BNO055(55, 0x28, &Wire);
}
void BNO::setupBNO(){
    adafruit_bno055_opmode_t mode = OPERATION_MODE_IMUPLUS;

    Serial.println("Orientation Sensor Test"); Serial.println("");

    // Initialise the sensor 
    if(!bno.begin())
    {
        // There was a problem detecting the BNO055 ... check your connections
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    delay(1000);
    bno.setExtCrystalUse(true);
}

void BNO::updateBNO(sensors_event_t &event){
    bno.getEvent(&event);
}
float BNO::getOrientationX(){
    updateBNO(event);
    return event.orientation.x;
}