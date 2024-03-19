int Encoder = 0;

void setup (){
    Serial.begin(115200);
    pinMode(39,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(39),captureCallback,RISING);
}  

void captureCallback(){
    
    Encoder++;
}

void loop (){
Serial.println(Encoder);
}
//496