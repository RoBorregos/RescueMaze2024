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

        static constexpr int kColorAmount_ = 3;

        static constexpr int kColorThresholdsAmount_ = 6;

        int colorAmount_ = 3;
        int precision_;
        const char *colorList_;
        float red_;
        float green_;
        float blue_;
        const int (*colors_)[kColorAmount_];

        const int (*colorThresholds_)[kColorThresholdsAmount_];

        const int kPrecision_ = 10;

        const int millisToWait_ = 50;

        const int kMaxRedValueInBlue_ = 110;
        const int kMinRedValueInBlue_ = 85;

        const int kMaxGreenValueInBlue_ = 140;
        const int kMinGreenValueInBlue_ = 80;

        const int kMaxBlueValueInBlue_ = 170;
        const int kMinBlueValueInBlue_ = 75;

        const int kMaxRedValueInRed_ = 270;
        const int kMinRedValueInRed_ = 150;

        const int kMaxGreenValueInRed_ = 80;
        const int kMinGreenValueInRed_ = 60;

        const int kMaxBlueValueInRed_ = 75;
        const int kMinBlueValueInRed_ = 50;

        const int kMinRedValueInBlack_ = 76;
        const int kMaxRedValueInBlack_ = 86;

        const int kMinGreenValueInBlack_ = 33;
        const int kMaxGreenValueInBlack_ = 56;

        const int kMinBlueValueInBlack_ = 30;
        const int kMaxBlueValueInBlack_ = 45;


        static constexpr char kRedColor_ = 'r';
        static constexpr char kBlueColor_ = 'b';
        static constexpr char kBlackColor_ = 'n';
        static constexpr char kUndifinedColor_ = 'u';

        bool inRange(uint8_t color, uint8_t colorRegistered);

        bool inRangeThreshold(double lowerBound, double colorDetection, double upperBound);

        void setDefaultValues();

    public:
        TCS();

        TCS(const uint8_t posMux);

        TCS(const uint8_t posMux, const int precision);

        void init();

        void init(const int colors[][3], const int colorAmount);

        void init(const int colors[][3], const int colorAmount, const char colorList[]);

        void init(const int colors[][3], const int colorAmount, const char colorList[], const int colorThresholds[][6]);

        void setMux(const uint8_t posMux);

        void setPrecision(const uint8_t precision);

        void printRGB();

        void printRGBC();

        void printColor();

        void updateRGB();

        void updateRGBC();

        char getColor();

        char getColorWithPrecision();
        
        char getColorWithThresholds();

        char getColorMode(const int sampleSize, const double certainity = 0);

        char getColorKReps(const int reps);

        void printColorMatrix();

        void printColorList();
};

#endif