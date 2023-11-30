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
    /* Serial.print("FL: ");
    Serial.print(robot.getMotorFL().getEncoderTicsFL());
    Serial.print("\t FR: ");
    Serial.print(robot.getMotorFR().getEncoderTicsFR());
    Serial.print("\t BL: ");
    Serial.print(robot.getMotorBL().getEncoderTicsBL());
    Serial.print("\t BR: ");
    Serial.print(robot.getMotorBR().getEncoderTicsBR());
    Serial.println();*/
    
    Serial.print(0);
    Serial.print(" ");
    Serial.print(180);
    Serial.print(" ");
    Serial.print(80);
    Serial.print(" ");
    Serial.print(robot.getPWMInicialFL());
    Serial.print(" ");
    Serial.print(robot.getPWMInicialFR());
    Serial.print(" ");
    Serial.print(robot.getPWMInicialBL());
    Serial.print(" ");
    Serial.println(robot.getPWMInicialBR());


/*     Serial.print(robot.getRPMFL());
    Serial.print(" ");
    Serial.print(" ");
    Serial.print(robot.getRPMFR());
    Serial.print(" ");
    Serial.print(" ");
    Serial.print(robot.getRPMBL());
    Serial.print(" ");
    Serial.println(robot.getRPMBR()); */
    //delay(250);
}