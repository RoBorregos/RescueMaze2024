#include "Movement.h"
#include "Motor.h"
#include "Pines.h"
#include "BNO.h"
#include "Encoder.h"

// #define BNO055_SAMPLERATE_DELAY_MS (100)

//BNO bno;
Movement robot;
PID pid(0.1,0.0,0.0);
double targetOrientation=0.0;




void setup()
{
    /* Serial.begin(115200);
    while (!Serial) delay(10); */  // wait for serial port to open!
    //robot.setup();
    //bno.setupBNO();

    // BIEEEN
    Serial.begin(115200);
    while (!Serial) delay(10); // wait for serial port to open!
    Encoder::initEncoder();
    //bno.setupBNO();
    robot.setup();	
    //targetOrientation = bno.getOrientationX();

    // esto tambien funciona 
    /* pinMode(17,OUTPUT);
    pinMode(5,OUTPUT);

    pinMode(18,OUTPUT);
    pinMode(19,OUTPUT);

    pinMode(25,OUTPUT);
    pinMode(33,OUTPUT);

    pinMode(32,OUTPUT);
    pinMode(14,OUTPUT); */


    /* analogWrite(23, 80);
    digitalWrite(18, HIGH);
    digitalWrite(19, LOW);

    analogWrite(26, 80);
    digitalWrite(25, HIGH);
    digitalWrite(33, LOW);

// BIEEEEEN
    analogWrite(27, 80);
    digitalWrite(32, HIGH);
    digitalWrite(14, LOW);
     */
}

void loop()
{
    //robot.setSpeed(80.00,360.00,bno);

    //robot.moveMotors(MotorState::Forward);

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
    robot.moveMotors(MotorState::Forward);

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