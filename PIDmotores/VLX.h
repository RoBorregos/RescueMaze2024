/* #ifndef VLX_h
#define VLX_h

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <MUX2C.h>

//COMPROBAR DIRECCION DEL VLX   
#define VLX_ADDR 0x29

class VLX{
    private:
        Adafruit_VL53L0X *vlx = Adafruit_VL53L0X();
        
        MUX2C *mux;

        //Almacenar datos de medicion de sensor
        VL53L0X_RangingMeasurementData_t measure;

        //conversion de milimetros a metros
        const float kMm_in_M= 0.001;
    

    public:
        VLX();
        VLX(unint8_t posMux);

        void setMux(unint8_t posMux);

        double getDistance();

        float getRawDistance();

        void printDistance();

};

#endif */