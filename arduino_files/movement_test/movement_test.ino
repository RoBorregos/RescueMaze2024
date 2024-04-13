const byte pwm_m1 = 26;
const byte m1_a = 14;
const byte m1_b = 27;

const byte pwm_m2 = 32;
const byte m2_a = 33;
const byte m2_b = 25;

const byte pwm_m3 = 15;
const byte m3_a = 4;
const byte m3_b = 2;

const byte pwm_m4 = 16;
const byte m4_a = 17;
const byte m4_b = 5;

int state_response = 0;

void avanzar(int pwm_set){
    analogWrite(pwm_m1, pwm_set);
    digitalWrite(m1_a, HIGH);
    digitalWrite(m1_b, LOW);
     analogWrite(pwm_m2, pwm_set);
    digitalWrite(m2_a, HIGH);
    digitalWrite(m2_b, LOW);
     analogWrite(pwm_m3, pwm_set);
    digitalWrite(m3_a, HIGH);
    digitalWrite(m3_b, LOW);
     analogWrite(pwm_m4, pwm_set);
    digitalWrite(m4_a, HIGH);
    digitalWrite(m4_b, LOW);

}

void stop(){
    analogWrite(pwm_m1,0);
    digitalWrite(m1_a, LOW);
    digitalWrite(m1_b, LOW);
     analogWrite(pwm_m2, 0);
    digitalWrite(m2_a, LOW);
    digitalWrite(m2_b, LOW);
     analogWrite(pwm_m3, 0);
    digitalWrite(m3_a, LOW);
    digitalWrite(m3_b, LOW);
     analogWrite(pwm_m4, 0);
    digitalWrite(m4_a, LOW);
    digitalWrite(m4_b, LOW);

}

void setup (){
    Serial.begin(115200);
    pinMode(pwm_m1, OUTPUT);
    pinMode(m1_a, OUTPUT);
    pinMode(m1_b, OUTPUT);
    pinMode(pwm_m2, OUTPUT);
    pinMode(m2_a, OUTPUT);
    pinMode(m2_b, OUTPUT);
    pinMode(pwm_m3, OUTPUT);
    pinMode(m3_a, OUTPUT);
    pinMode(m3_b, OUTPUT);
    pinMode(pwm_m4, OUTPUT);
    pinMode(m4_a, OUTPUT);
    pinMode(m4_b, OUTPUT);



}

void loop () {
    
    // Serial.print('1'); // Send the byte 'A'
    // delay(1000); // Wait for a response
//   while(!Serial.available()) { // Wait for a response
//     delay(10); // Delay a little bit to not stress the CPU
//   }

//   while(Serial.available() && state_response == 0) { // If data is available to read,
//     char received = Serial.read(); // read it and store it in 'received'
//     if(received == 'm') { // If the received byte is 'A'
//       state_response = 0; // Set the variable to 0
//     }
//     else{ // If the received byte is 'B'
//       state_response = 1; // Set the variable to 1
//     }
//   }
  
// if (state_response != 0){
//     stop();
// }
// else{
//     avanzar(120);
// }

}