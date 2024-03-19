#ifndef TCS_h
#define TCS_h

#include "Adafruit_TCS34725.h"
#include "MUX.h"
#include "CustomSerial.h"

// TODO: check which address is the correct one
#define TCS_ADDR 0x30 

class TCS {
    private:
        Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
        MUX mux;
        int colors = 3;
        int precision;
        const char *colorList;
        double red;
        double green;
        double blue;
        const int (*colors)[3];

        const int (*colorThresholds)[6];

        bool inRange(uint8_t color, uint8_t colorRegistered);

        bool inRangeThreshold(double lowerBound, double colorDetection, double upperBound);

        void setDefaultValues();

    public:
        TCS();

        TCS(uint8_t posMux);

        TCS(uint8_t posMux, uint8_t precision);

        void init();

        void init(const uint8_t colors[][3], const uint8_t colorAmount);

        void init(const char colors[][3], const uint8_t colorAmount, const char colorList[], const int colorThresholds[][6]);

        void setMux(uint8_t posMux);

        void setPrecision(uint8_t precision);

        void printRGB();

        void printRGBC();

        void printColor();

        void updateRGB();

        void updateRGBC();

        char getColor();

        char getColorWithPrecision();
        
        char getColorWithThresholds();

        char getColorMode(int sampleSize, double certainity = 0);

        char getColorKReps(int reps);

        void printColorMatrix();

        void printColorList();
}

#endif