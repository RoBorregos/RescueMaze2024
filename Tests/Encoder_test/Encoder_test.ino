int Encoder = 0;

void setup (){
    Serial.begin(115200);
    pinMode(35,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(35),captureCallback,RISING);
}  

void captureCallback(){
    
    Encoder++;
}

void loop (){
Serial.println(Encoder);
}
//496