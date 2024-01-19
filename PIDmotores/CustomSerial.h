// TODO: THIS DOESN'T WORK FOR ALL FILES ONLY FOR .INO
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