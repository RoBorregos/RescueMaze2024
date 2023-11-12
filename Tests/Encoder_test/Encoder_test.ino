int Encoder = 0;

void setup (){
    Serial.begin(115200);
    pinMode(2,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2),captureCallback,RISING);
}  

void captureCallback(){
    Serial.println(Encoder);
    Encoder++;
}

void loop (){

}
//496