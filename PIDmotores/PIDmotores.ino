#include "Movement.h"
//#include "Motor.h"
//#include "Pines.h"
#include "BNO.h"
//#include "encoder.h"

//#define BNO055_SAMPLERATE_DELAY_MS (100)

BNO bno;
Movement robot;

void setup()
{
    Serial.begin(115200);
    robot.setup();
    bno.setupBNO();
}

void loop()
{
    robot.setSpeed(80.00,360.00,bno);
    /*Serial.print("FL: ");
    Serial.print(robot.getMotorFL().getEncoderTicsFL());
    Serial.print("\t FR: ");
    Serial.print(robot.getMotorFR().getEncoderTicsFR());
    Serial.print("\t BL: ");
    Serial.print(robot.getMotorBL().getEncoderTicsBL());
    Serial.print("\t BR: ");
    Serial.print(robot.getMotorBR().getEncoderTicsBR());
    Serial.println();*/
    /*Serial.print("FL: ");
    Serial.print(robot.getRMPFL());
    Serial.print("\t FR: ");
    Serial.print(robot.getRMPFR());
    Serial.print("\t BL: ");
    Serial.print(robot.getRMPBL());
    Serial.print("\t BR: ");
    Serial.print(robot.getRMPBR());
    Serial.println();*/
    //delay(250);
}