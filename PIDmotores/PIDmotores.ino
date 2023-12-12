#include "Movement.h"
#include "Motor.h"
#include "Pines.h"
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
}

void loop()
{
    robot.moveMotors(MotorState::Forward);
    
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
    //robot.moveMotors(MotorState::Forward);

    /* delay(2000);

    robot.moveMotors(MotorState::Stop);

    delay(2000);

    robot.moveMotors(MotorState::Backward);

    
 */


    // FUNCIONAL MOVERSE PARA ADELANTE SIN NINUGNA FUNCION AUNQUE SE VA INICIANDO UNO A UNO 
/*     analogWrite(16, 80);
    digitalWrite(17, HIGH);
    digitalWrite(5, LOW);

    delay(2000);

    analogWrite(23, 80);
    digitalWrite(18, HIGH);
    digitalWrite(19, LOW);

    delay(2000);

    analogWrite(26, 80);
    digitalWrite(25, HIGH);
    digitalWrite(33, LOW);
    
    delay(2000);

    analogWrite(27, 80);
    digitalWrite(32, HIGH);
    digitalWrite(14, LOW); */
    

    //robot.moveForward(100,100,100,100);
    //delay(250);
}