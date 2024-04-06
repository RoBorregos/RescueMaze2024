int Encoder = 0;
int Encoder_2 = 0;
int Encoder_3 = 0;
int Encoder_4 = 0;
void setup (){
    Serial.begin(115200);
    pinMode(35,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(35),captureCallback,RISING);
    pinMode(34,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(34),captureCallback_2,RISING);
    pinMode(36,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(36),captureCallback_3,RISING);
    pinMode(39,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(39),captureCallback_4,RISING);
}  

void captureCallback(){
    
    Encoder++;
}

void captureCallback_2(){
    
    Encoder_2++;
}
void captureCallback_3(){
    
    Encoder_3++;
}
void captureCallback_4(){
    
    Encoder_4++;
}
void loop (){
Serial.print("Encoder 1: ");
Serial.print(Encoder);
Serial.print(" Encoder 2: ");
Serial.print(Encoder_2);
Serial.print(" Encoder 3: ");
Serial.print(Encoder_3);
Serial.print(" Encoder 4: ");
Serial.println(Encoder_4);


}
//496

//viendo desde atras del robot
//36 izquierda adelante
//39 derecha adelante
//34 derecha atras
//