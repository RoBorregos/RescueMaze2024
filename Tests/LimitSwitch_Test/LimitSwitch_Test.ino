void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);

}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println(getState(13));
  Serial.println(getState(23));


}


bool getState(int pin_) {
    bool state_;
    const uint8_t val = digitalRead(pin_);
    if (val == HIGH) {
        #if DEBUG_LIMIT_SWITCH
        customPrintln("LimitSwitch is active");
        #endif
        state_ = true;
    } else {
        state_ = false;
    }
    return state_;
}