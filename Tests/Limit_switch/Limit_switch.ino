void setup(){
    Serial.begin(115200);
    pinMode(13,INPUT_PULLUP);
}
void loop(){
    Serial.println(digitalRead(13));
    delay(100);
}