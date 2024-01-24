#include "CustomSerial.h"
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
    //robot.setSpeed(1);
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
    customPrint("BACK_LEFT: ");
    customPrintln(robot.getBackLeftSpeed());
    customPrint("FRONT_LEFT: ");
    customPrintln(robot.getFrontLeftSpeed());
    customPrint("BACK_RIGHT: ");
    customPrintln(robot.getBackRightSpeed());
    customPrint("FRONT_RIGHT: ");
    customPrintln(robot.getFrontRightSpeed()); 

    delay(200);

    
    
    
    // Asi se mueven los motores sin PID
    //robot.moveMotors(MotorState::kForward);

    /* delay(2000);

    robot.moveMotors(MotorState::kStop);

    delay(2000);

    robot.moveMotors(MotorState::kBackward); 
 */
}