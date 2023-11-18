#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "motor.h"
#include "encoder.h"


/*#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);*/

void setup(void)
{
    Serial.begin(115200);

    motor motorFL;
    motorFL.motoresSetup(7, 29, 28, 3); //, "FRONT_LEFT"

    motor motorFR;
    motorFR.motoresSetup(6, 27, 26, 2); //, "FRONT_RIGHT"

    motor motorBL;
    motorBL.motoresSetup(4, 23, 22, 19); //, "BACK_LEFT"

    motor motorBR;
    motorBR.motoresSetup(5, 25, 24, 18); //, "BACK_RIGHT"

    /*while (!Serial) delay(10);  // wait for serial port to open!

    Serial.println("Orientation Sensor Test"); Serial.println("");*/

    // Initialise the sensor 
    /*if(!bno.begin())
    {
        //There was a problem detecting the BNO055 ... check your connections
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    delay(1000);
    bno.setExtCrystalUse(true);*/
}

void loop(void)
{
    /*sensors_event_t event;
    bno.getEvent(&event);*/
    /*
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    */
    /*Serial.print(encoderFL.encoderTics,"\t");
    Serial.print(encoderFR.encoderTics,"\t");
    Serial.print(encoderBL.encoderTics,"\t");
    Serial.print(encoderBR.encoderTics,"\n");*/
    //delay(BNO055_SAMPLERATE_DELAY_MS);
}
//pq 4 funciones para encoders