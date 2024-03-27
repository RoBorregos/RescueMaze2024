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


        int8_t colorAmount_ = 3;
        int8_t precision_;
        const char *colorList_;
        float red_;
        float green_;
        float blue_;
        const int16_t (*colors_)[kColorAmount_];

        static constexpr int8_t kColorThresholdsAmount_ = 6;
        const int16_t (*colorThresholds_)[kColorThresholdsAmount_];

        static constexpr int8_t kColumnAmount_ = 3;


        static constexpr int8_t kPrecision_ = 10;

        static constexpr int8_t millisToWait_ = 50;

        static constexpr int16_t kMaxRedValueInBlue_ = 110;
        static constexpr int16_t kMinRedValueInBlue_ = 85;

        static constexpr int16_t kMaxGreenValueInBlue_ = 140;
        static constexpr int16_t kMinGreenValueInBlue_ = 80;

        static constexpr int16_t kMaxBlueValueInBlue_ = 170;
        static constexpr int16_t kMinBlueValueInBlue_ = 75;

        static constexpr int16_t kMaxRedValueInRed_ = 270;
        static constexpr int16_t kMinRedValueInRed_ = 150;

        static constexpr int16_t kMaxGreenValueInRed_ = 80;
        static constexpr int16_t kMinGreenValueInRed_ = 60;

        static constexpr int16_t kMaxBlueValueInRed_ = 75;
        static constexpr int16_t kMinBlueValueInRed_ = 50;

        static constexpr int16_t kMinRedValueInBlack_ = 76;
        static constexpr int16_t kMaxRedValueInBlack_ = 86;

        static constexpr int16_t kMinGreenValueInBlack_ = 33;
        static constexpr int16_t kMaxGreenValueInBlack_ = 56;

        static constexpr int16_t kMinBlueValueInBlack_ = 30;
        static constexpr int16_t kMaxBlueValueInBlack_ = 45;


        static constexpr char kRedColor_ = 'r';
        static constexpr char kBlueColor_ = 'b';
        static constexpr char kBlackColor_ = 'n';
        static constexpr char kUndefinedColor_ = 'u';

        bool inRange(const uint8_t color, const uint8_t colorRegistered);

        bool inRangeThreshold(const double lowerBound, const double colorDetection, const double upperBound);

        void setDefaultValues();

    public:
        TCS();

        TCS(const uint8_t posMux);

        TCS(const uint8_t posMux, const int precision);

        void init();

        void init(const int16_t colors[][kColumnAmount_], const int8_t colorAmount, const char colorList[], const int16_t colorThresholds[][kColorThresholdsAmount_]);

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