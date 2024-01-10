#include "Movement.h"
#include "Motor.h"
#include "Pins.h"
#include "BNO.h"
#include "Encoder.h"

Movement robot;
double targetOrientation = 0.0;
unsigned long iterations = 0;

void setup(){
    Serial.begin(115200);
    while (!Serial) delay(10); // wait for serial port to open!
    robot.setup();
}
    
void loop() {
    robot.moveMotors(MovementState::kForward, 0);
    
    /* if (iterations < 25) { // TODO: juega con este numero
        robot.moveMotors(MovementState::kForward, 0);
    } else if (iterations < 50) { // TODO: juega con este numero
        robot.moveMotors(MovementState::kStop, 0);
    } else if (iterations < 75) {
        robot.moveMotors(MovementState::kBackward, 0);
    } else if (iterations < 100) {
        robot.moveMotors(MovementState::kStop, 0);
    } else if (iterations < 125) {
        robot.moveMotors(MovementState::kTurnLeft, 90);
    } else if (iterations < 150) {
        robot.moveMotors(MovementState::kStop, 0);
    } else if (iterations < 175) {
        robot.moveMotors(MovementState::kTurnRight, 270);
    }
    else {
        robot.moveMotors(MovementState::kStop, 0);
        delay(10000);
    } */
    
    ++iterations;

    //LEER TICS DE LOS ENCODERS
    Serial.print("BACK_LEFT: ");
    Serial.println(robot.getBackLeftSpeed());
    Serial.print("FRONT_LEFT: ");
    Serial.println(robot.getFrontLeftSpeed());
    Serial.print("BACK_RIGHT: ");
    Serial.println(robot.getBackRightSpeed());
    Serial.print("FRONT_RIGHT: ");
    Serial.println(robot.getFrontRightSpeed()); 

    delay(200);

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