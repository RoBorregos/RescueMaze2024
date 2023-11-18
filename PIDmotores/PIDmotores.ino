#include "Motor.h"
#include "Pines.h"
#include "BNO.h"
//#include "encoder.h"

//#define BNO055_SAMPLERATE_DELAY_MS (100)

Motor motorFL;
Motor motorFR;
Motor motorBL;
Motor motorBR;
//BNO bno;

void setup()
{
    Serial.begin(115200);

    motorFL.motoresSetup(pwmFL, daFL, dbFL, eaFL, MotorID::FRONT_LEFT); 
    motorFR.motoresSetup(pwmFR, daFR, dbFR, eaFR, MotorID::FRONT_RIGHT); 
    motorBL.motoresSetup(pwmBL, daBL, dbBL, eaBL, MotorID::BACK_LEFT); 
    motorBR.motoresSetup(pwmBR, daBR, dbBR, eaBR, MotorID::BACK_RIGHT);

    //bno.setupBNO();
}

void loop()
{
    Serial.print("FL\tFR\tBL\tBR\n");
    Serial.print(motorFL.getEncoderTicsFL());
    Serial.print("\t");
    Serial.print(motorFR.getEncoderTicsFR());
    Serial.print("\t");
    Serial.print(motorBL.getEncoderTicsBL());
    Serial.print("\t");
    Serial.print(motorBR.getEncoderTicsBR());
    Serial.println();
    //delay(100);
}