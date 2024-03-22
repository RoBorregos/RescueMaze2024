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

        static constexpr int kColumnAmount_ = 3;

        static constexpr int kPrecision_ = 10;

        static constexpr int millisToWait_ = 50;

        static constexpr int kMaxRedValueInBlue_ = 110;
        static constexpr int kMinRedValueInBlue_ = 85;

        static constexpr int kMaxGreenValueInBlue_ = 140;
        static constexpr int kMinGreenValueInBlue_ = 80;

        static constexpr int kMaxBlueValueInBlue_ = 170;
        static constexpr int kMinBlueValueInBlue_ = 75;

        static constexpr int kMaxRedValueInRed_ = 270;
        static constexpr int kMinRedValueInRed_ = 150;

        static constexpr int kMaxGreenValueInRed_ = 80;
        static constexpr int kMinGreenValueInRed_ = 60;

        static constexpr int kMaxBlueValueInRed_ = 75;
        static constexpr int kMinBlueValueInRed_ = 50;

        static constexpr int kMinRedValueInBlack_ = 76;
        static constexpr int kMaxRedValueInBlack_ = 86;

        static constexpr int kMinGreenValueInBlack_ = 33;
        static constexpr int kMaxGreenValueInBlack_ = 56;

        static constexpr int kMinBlueValueInBlack_ = 30;
        static constexpr int kMaxBlueValueInBlack_ = 45;


        static constexpr char kRedColor_ = 'r';
        static constexpr char kBlueColor_ = 'b';
        static constexpr char kBlackColor_ = 'n';
        static constexpr char kUndifinedColor_ = 'u';

        bool inRange(const uint8_t color, const uint8_t colorRegistered);

        bool inRangeThreshold(const double lowerBound, const double colorDetection, const double upperBound);

        void setDefaultValues();

    public:
        TCS();

        TCS(const uint8_t posMux);

        TCS(const uint8_t posMux, const int precision);

        void init();

        void init(const int colors[][kColumnAmount_], const int colorAmount);

        void init(const int colors[][kColumnAmount_], const int colorAmount, const char colorList[]);

        void init(const int colors[][kColumnAmount_], const int colorAmount, const char colorList[], const int colorThresholds[][6]);

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