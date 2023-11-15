#include <Arduino.h>    


// Definición de la clase PID
class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd)
    : kp(Kp), ki(Ki), kd(Kd), lastError(0), integral(0) {}

    double compute(double setpoint, double measuredValue) {
        double error = setpoint - measuredValue;
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        return kp * error + ki * integral + kd * derivative;
    }

private:
    double kp, ki, kd;
    double lastError, integral;
};

// Pines para el controlador L298N
const int motorEnablePin = 9; // Pin de habilitación (PWM)
const int motorInput1 = 10;   // Pin de dirección 1
const int motorInput2 = 11;   // Pin de dirección 2

// Pines y variables para el encoder
const int encoderPinA = 2;
volatile long encoderTicks = 0;

// Parámetros del PID
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PIDController pid(Kp, Ki, Kd);

void setup() {
     pinMode(motorEnablePin, OUTPUT);
    pinMode(motorInput1, OUTPUT);
    pinMode(motorInput2, OUTPUT);

    pinMode(encoderPinA, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);
}

void loop() {
    long currentTime = millis();
    double deltaTime = (currentTime - lastMeasurementTime) / 1000.0; // Tiempo en segundos
    if (deltaTime >= 0.01) { // Actualiza cada 10 ms
        double rpm = (encoderTicks / 496.0) / deltaTime; // Calcula RPM (ajusta 496 según tu sistema)
        encoderTicks = 0;
        lastMeasurementTime = currentTime;

        double motorSpeed = pid.compute(100, rpm); // Setpoint de 100 RPM como ejemplo
        controlMotor(motorSpeed);
    }
}
void encoderISR() {
    encoderTicks++;
}

void controlMotor(double output) {
    int pwmValue = constrain(output, 0, 255);

    // Establece la dirección del motor
    if (pwmValue > 0) {
        digitalWrite(motorInput1, HIGH);
        digitalWrite(motorInput2, LOW);
    } else {
        digitalWrite(motorInput1, LOW);
        digitalWrite(motorInput2, HIGH);
        pwmValue = -pwmValue; // Invierte el valor para la dirección opuesta
    }

    analogWrite(motorEnablePin, pwmValue);
}