//#include "Motor.h"
//#include "Pines.h"
//#include "BNO.h"
#include "Movement.h"
//#include "encoder.h"

//#define BNO055_SAMPLERATE_DELAY_MS (100)

/*Motor motorFL;
Motor motorFR;
Motor motorBL;
Motor motorBR;*/
//BNO bno;
Movement robot;

unsigned long next_time;

void setup()
{
    Serial.begin(115200);
    robot.setup();
    //bno.setupBNO();
    next_time = millis();
}

void loop()
{
    if(millis()-next_time>=1000){
        robot.updateRPM();
        next_time = millis();
    }
    robot.moveForward(100);
    /*Serial.print("FL: ");
    Serial.print(robot.getMotorFL().getEncoderTicsFL());
    Serial.print("\t FR: ");
    Serial.print(robot.getMotorFR().getEncoderTicsFR());
    Serial.print("\t BL: ");
    Serial.print(robot.getMotorBL().getEncoderTicsBL());
    Serial.print("\t BR: ");
    Serial.print(robot.getMotorBR().getEncoderTicsBR());
    Serial.println();*/
    Serial.print("FL: ");
    Serial.print(robot.getRMPFL());
    Serial.print("\t FR: ");
    Serial.print(robot.getRMPFR());
    Serial.print("\t BL: ");
    Serial.print(robot.getRMPBL());
    Serial.print("\t BR: ");
    Serial.print(robot.getRMPBR());
    Serial.println();
    //delay(250);
}