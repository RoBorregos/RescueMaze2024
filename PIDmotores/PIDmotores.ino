#include "Movement.h"
// #include "Motor.h"
// #include "Pines.h"
#include "BNO.h"
// #include "encoder.h"

// #define BNO055_SAMPLERATE_DELAY_MS (100)

BNO bno;
Movement robot;

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(10);  // wait for serial port to open!
    robot.setup();
    bno.setupBNO();
    
}

void loop()
{
    robot.setSpeed(80.00,360.00,bno);
    //robot.moveForward(100,100,100,100);
    /*Serial.print(robot.getMotorFR().getEncoderTics());
    Serial.print(" ");
    Serial.print(robot.getMotorFL().getEncoderTics());
    Serial.print(" ");
    Serial.print(robot.getMotorBR().getEncoderTics());
    Serial.print(" ");
    Serial.println(robot.getMotorBL().getEncoderTics());*/
    /*Serial.print(0);
    Serial.print(" ");
    Serial.print(180);
    Serial.print(" ");
    Serial.print(80);
    Serial.print(" ");*/
    /*Serial.print(robot.getMotorFL().getPWM());
    Serial.print(" ");
    Serial.print(robot.getMotorFR().getPWM());
    Serial.print(" ");
    Serial.print(robot.getMotorBL().getPWM());
    Serial.print(" ");
    Serial.print(robot.getMotorBR().getPWM());
    Serial.print(" ");*/
    /*Serial.print(robot.getMotorFL().getRPM());
    Serial.print(" ");
    Serial.print(robot.getMotorFR().getRPM());
    Serial.print(" ");
    Serial.print(robot.getMotorBL().getRPM());
    Serial.print(" ");
    Serial.println(robot.getMotorBR().getRPM());*/

    //delay(250);
}