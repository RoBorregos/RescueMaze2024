int Encoder = 0;

void setup (){
    Serial.begin(115200);
    pinMode(19,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(19),captureCallback,RISING);
}  

void captureCallback(){
    Serial.println(Encoder);
    Encoder++;
}

void loop (){

}
//496