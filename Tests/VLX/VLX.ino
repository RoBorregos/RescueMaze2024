/* #include <Arduino.h>
#include <Wire.h>

uint8_t tcaPos_ = 0;
void setNewChannel(uint8_t tcaPos) {
    tcaPos_ = tcaPos;
}

void selectChannel(uint8_t tcaPos) {
    setNewChannel(tcaPos);
    selectChannel();
}

void selectChannel() {
    Wire.beginTransmission(0x70);
    Wire.write(1 << tcaPos_);
    Wire.endTransmission();
}

void findI2C(bool scan, uint8_t address) {
    if (scan) {
        Wire.begin();
        for (uint8_t i = 0; i < 128; i++) {
            Wire.beginTransmission(i);
            if (Wire.endTransmission() == 0) {
                Serial.print("Found address: ");
                Serial.println(i, DEC);
            }
        }
    } else {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.print("Found address: ");
            Serial.println(address, DEC);
        }
    }
}

void setMatchingI2C(uint8_t address) {
    findI2C(false, address);
}

uint8_t getTcaPos() {
    return tcaPos_;
}

VLX(uint8_t posMux) {
    mux_.setNewChannel(posMux);
}

void setMux(uint8_t posMux) {
    mux_.setNewChannel(posMux);
}

void init() {
    mux_.selectChannel();
    if (!vLX_.begin()){
        customPrintln("ERROR VLX");
    }
}

float getRawDistance() {
    mux_.selectChannel();
    vLX_.rangingTest(&measure_, false);

    return measure_.RangeMilliMeter;
}

double getDistance() {
    mux_.selectChannel();
    vLX_.rangingTest(&measure_, false);
    double measure = (measure_.RangeMilliMeter / 1000.00);
    singleEMAFilter.AddValue(measure);
    
    return singleEMAFilter.GetLowPass();
}

void printDistance() {
    customPrint("Distance (M): ");
    customPrintln(getDistance());
} */

#include <Wire.h>
#include "VLX.h"
#include "MUX.h"

MUX muxInstance;
VLX vlxInstance(1);

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    // Inicializar la instancia del MUX (puedes ajustar el número del canal según tu necesidad)
  
    // Inicializar la instancia del VLX
    vlxInstance.init();
}

void loop() {
    // Obtener y imprimir la distancia desde el sensor VLX
    vlxInstance.getDistance();
    Serial.println(vlxInstance.getDistance());

    delay(1000);  // Puedes ajustar el tiempo de espera según tus necesidades
}