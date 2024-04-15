#include <Arduino.h>

// #include <iostream>
// #include <wiringPi.h>

// Define los pines del sensor ultrasónico
// const int TRIG_PIN = 18;
// const int ECHO_PIN = 24;

// void setup() {
//     wiringPiSetup();
//     pinMode(TRIG_PIN, OUTPUT);
//     pinMode(ECHO_PIN, INPUT);
// }

// float measureDistance() {
//     // Envía un pulso al TRIG_PIN
//     digitalWrite(TRIG_PIN, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(TRIG_PIN, LOW);

//     // Mide el tiempo que tarda el eco en regresar
//     while (digitalRead(ECHO_PIN) == LOW) {
//         // Espera a que el eco comience
//     }
//     long startTime = micros();

//     while (digitalRead(ECHO_PIN) == HIGH) {
//         // Espera a que el eco termine
//     }
//     long endTime = micros();

//     // Calcula la distancia en centímetros
//     float duration = static_cast<float>(endTime - startTime);
//     float distance = duration * 0.0343 / 2.0; // Velocidad del sonido en el aire: 343 m/s
//     return distance;
// }

// int main() {
//     setup();

//     while (true) {
//         float dist = measureDistance();
//         std::cout << "Distancia: " << dist << " cm" << std::endl;
//         delay(1000);
//     }

//     return 0;
// }


// #include <QTRSensors.h>

// // Configura el número de sensores y los pines a los que están conectados
// const uint8_t numSensors = 8;
// const uint8_t sensorPins[numSensors] = {2, 3, 4, 5, 6, 7, 8, 9}; // Ejemplo de pines, ajusta según tu configuración

// // Crea un objeto QTRSensors
// QTRSensorsRC qtrrc((unsigned char[])sensorPins, numSensors);

// void setup() {
//     // Inicializa el sensor
//     qtrrc.init();

//     // Calibra el sensor (debes seguir las instrucciones para calibrar correctamente)
//     qtrrc.calibrate(QTR_EMITTERS_ON);

//     // Espera un breve tiempo para que el sensor se estabilice
//     delay(500);
// }

// void loop() {
//     unsigned int sensorValues[numSensors];
//     qtrrc.read(sensorValues);

//     // Imprime los valores de reflectancia
//     for (uint8_t i = 0; i < numSensors; i++) {
//         Serial.print(sensorValues[i]);
//         Serial.print("\t");
//     }
//     Serial.println();

//     // Puedes utilizar los valores de reflectancia para tomar decisiones en tu aplicación
//     // Por ejemplo, detectar una línea negra sobre un fondo blanco

//     delay(100); // Espera antes de la siguiente lectura
// }


/* #include <QTRSensors.h>

// Configura el número de sensores y los pines a los que están conectados
const uint8_t numSensors = 8;
const uint8_t sensorPins[numSensors] = {2, 3, 4, 5, 6, 7, 8, 9}; // Ejemplo de pines, ajusta según tu configuración

// Crea un objeto QTRSensors
QTRSensorsRC qtrrc((unsigned char[])sensorPins, numSensors);

void setup() {
    // Inicializa el sensor
    qtrrc.init();

    // Calibra el sensor (debes seguir las instrucciones para calibrar correctamente)
    qtrrc.calibrate(QTR_EMITTERS_ON);

    // Espera un breve tiempo para que el sensor se estabilice
    delay(500);
}

void loop() {
    unsigned int sensorValues[numSensors];
    qtrrc.read(sensorValues);

    // Imprime los valores de reflectancia
    for (uint8_t i = 0; i < numSensors; i++) {
        Serial.print(sensorValues[i]);
        Serial.print("\t");
    }
    Serial.println();

    // Puedes utilizar los valores de reflectancia para tomar decisiones en tu aplicación
    // Por ejemplo, detectar una línea negra sobre un fondo blanco

    delay(100); // Espera antes de la siguiente lectura
} */


/* // Especificamos el PIN al que se conectó el OUT del sensor
const int sensorPin = A0;

void setup() {
  pinMode(sensorPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  int sensorValue = digitalRead(sensorPin);
  if (sensorValue == HIGH) {
    Serial.println("Obstáculo detectado");
  } else {
    Serial.println("Sin obstáculos");
  }
  delay(1000);
}
 */

void setup(){
    pinMode(33,INPUT);
    Serial.begin(9600);
}

void loop() {
    Serial.println(analogRead(33));
    delay(100);
}