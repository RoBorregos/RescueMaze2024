//pines de motores
const int pwmRear_a = 2;
const int derRear_a = 22;
const int derRear_b = 23;
const int pwmRear_b = 3;
const int izqRear_a = 25;
const int izqRear_b = 24;
const int pwmFront_a = 4;
const int derFront_a = 28;
const int derFront_b = 29;
const int pwmFront_b = 5;
const int izqFront_a = 27;
const int izqFront_b = 26;
const int pwm = 90;

void avanzar(){
    analogWrite(pwmRear_a,pwm);
    digitalWrite(derRear_a,HIGH);
    digitalWrite(derRear_b,LOW);
    analogWrite(pwmRear_b,pwm);
    digitalWrite(izqRear_a,HIGH);
    digitalWrite(izqRear_b,LOW);
    analogWrite(pwmFront_a,pwm);
    digitalWrite(derFront_a,HIGH);
    digitalWrite(derFront_b,LOW);
    analogWrite(pwmFront_b,pwm);
    digitalWrite(izqFront_a,HIGH);
    digitalWrite(izqFront_b,LOW);
}
void girarDer(){
    analogWrite(pwmRear_a,pwm);
    digitalWrite(derRear_a,LOW);
    digitalWrite(derRear_b,HIGH);
    analogWrite(pwmRear_b,pwm);
    digitalWrite(izqRear_a,HIGH);
    digitalWrite(izqRear_b,LOW);
    analogWrite(pwmFront_a,pwm);
    digitalWrite(derFront_a,LOW);
    digitalWrite(derFront_b,HIGH);
    analogWrite(pwmFront_b,pwm);
    digitalWrite(izqFront_a,HIGH);
    digitalWrite(izqFront_b,LOW);
}
void girarIzq(){
    analogWrite(pwmRear_a,pwm);
    digitalWrite(derRear_a,HIGH);
    digitalWrite(derRear_b,LOW);
    analogWrite(pwmRear_b,pwm);
    digitalWrite(izqRear_a,LOW);
    digitalWrite(izqRear_b,HIGH);
    analogWrite(pwmFront_a,pwm);
    digitalWrite(derFront_a,HIGH);
    digitalWrite(derFront_b,LOW);
    analogWrite(pwmFront_b,pwm);
    digitalWrite(izqFront_a,LOW);
    digitalWrite(izqFront_b,HIGH);
}
void atras(){
    analogWrite(pwmRear_a,pwm);
    digitalWrite(derRear_a,LOW);
    digitalWrite(derRear_b,HIGH);
    analogWrite(pwmRear_b,pwm);
    digitalWrite(izqRear_a,LOW);
    digitalWrite(izqRear_b,HIGH);
    analogWrite(pwmFront_a,pwm);
    digitalWrite(derFront_a,LOW);
    digitalWrite(derFront_b,HIGH);
    analogWrite(pwmFront_b,pwm);
    digitalWrite(izqFront_a,LOW);
    digitalWrite(izqFront_b,HIGH);
}
void parar(){
    analogWrite(pwmRear_a,0);
    digitalWrite(derRear_a,LOW);
    digitalWrite(derRear_b,HIGH);
    analogWrite(pwmRear_b,0);
    digitalWrite(izqRear_a,LOW);
    digitalWrite(izqRear_b,HIGH);
    analogWrite(pwmFront_a,0);
    digitalWrite(derFront_a,LOW);
    digitalWrite(derFront_b,HIGH);
    analogWrite(pwmFront_b,0);
    digitalWrite(izqFront_a,LOW);
    digitalWrite(izqFront_b,HIGH);
}
void setup (){
    Serial.begin(115200);
    pinMode(pwmRear_a, OUTPUT);
    pinMode(derRear_a, OUTPUT);
    pinMode(derRear_b, OUTPUT);
    pinMode(pwmRear_b, OUTPUT);
    pinMode(izqRear_a, OUTPUT);
    pinMode(izqRear_b, OUTPUT);
    pinMode(pwmFront_a, OUTPUT);
    pinMode(derFront_a, OUTPUT);
    pinMode(derFront_b, OUTPUT);
    pinMode(pwmFront_b, OUTPUT);
    pinMode(izqFront_a, OUTPUT);
    pinMode(izqFront_b, OUTPUT);
}
void loop(){
    avanzar();
    delay(1000);
    parar();
    delay(1000);
    girarDer();
    delay(1000);
    parar();
    delay(1000);
    girarIzq();
    delay(1000);
    parar();
    delay(1000);
    atras();
    delay(1000);
    parar();
    delay(1000);
}