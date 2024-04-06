#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
uint16_t r, g, b, c;

#define TCAADDR 0x70
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


void leerColor(){ 
    //rutina para leer color
    tcs.getRawData(&r, &g, &b, &c);
    Serial.print(r);
    Serial.print(" ");
    Serial.print(g);
    Serial.print(" ");
    Serial.print(b);
    Serial.print(" ");
    Serial.println(c);
}

void setup(void) {
  Serial.begin(115200);
  Wire.begin();
  tcaselect(1);
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}


void loop(void) {
  leerColor();
}