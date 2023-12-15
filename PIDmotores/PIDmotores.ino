#include "Movement.h"
#include "Motor.h"
#include "Pins.h"
#include "BNO.h"
#include "Encoder.h"

Movement robot;
double targetOrientation=0.0;

void setup()
{
    // BIEEEN
    Serial.begin(115200);
    while (!Serial) delay(10); // wait for serial port to open!
    robot.setup();	
    robot.moveMotors(MovementState::kTurnLeft, 270);
    delay(1000);
    robot.moveMotors(MovementState::kTurnRight, 0);
    delay(1000);
}

void loop()
{
    
    
    
    
    //LEER TICS DE LOS ENCODERS
    /* Serial.print("BACK_LEFT: ");
    Serial.println(robot.getBackLeftEncoderTics());
    Serial.print("FRONT_LEFT: ");
    Serial.println(robot.getFrontLeftEncoderTics());
    Serial.print("BACK_RIGHT: ");
    Serial.println(robot.getBackRightEncoderTics());
    Serial.print("FRONT_RIGHT: ");
    Serial.println(robot.getFrontRightEncoderTics()); */

    // Asi se leen los valores del eje x del bno
    /* double orientacionX = bno.getOrientationX();
    Serial.print("orientacionX: ");
    Serial.println(orientacionX); */
    
    // Asi se mueven los motores sin PID
    //robot.moveMotors(MotorState::kForward);

    /* delay(2000);

    robot.moveMotors(MotorState::kStop);

    delay(2000);

    robot.moveMotors(MotorState::kBackward); 
 */
}