#ifndef CUSTOMSERIAL_H
#define CUSTOMSERIAL_H
// What does PRINT_SERIAL do? Well this can be used to print to the serial monitor only if it's defined
// #define PRINT_SERIAL
template <typename T>
void customPrintln(T message) {
    #ifdef PRINT_SERIAL
    Serial.println(message);
    #endif
}

template <typename T>
void customPrint(T message) {
    #ifdef PRINT_SERIAL
    Serial.print(message);
    #endif
}

#endif