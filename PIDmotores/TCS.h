#ifndef TCS_h
#define TCS_h

#include "Adafruit_TCS34725.h"
#include "MUX.h"
#include "CustomSerial.h"

// TODO: check which address is the correct one
#define TCS_ADDR 0x30 

class TCS {
    private:
        Adafruit_TCS34725 tcs_ = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
        MUX mux_;

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

        float kMaxRedValueInBlue_ = 85;
        float kMinRedValueInBlue_ = 85;

        float kMaxGreenValueInBlue_ = 140;
        float kMinGreenValueInBlue_ = 80;

        float kMaxBlueValueInBlue_ = 170;
        float kMinBlueValueInBlue_ = 100;

        float kMaxRedValueInRed_ = 270;
        float kMinRedValueInRed_ = 150;

        float kMaxGreenValueInRed_ = 80;
        float kMinGreenValueInRed_ = 60;

        float kMaxBlueValueInRed_ = 75;
        float kMinBlueValueInRed_ = 50;

        float kMinRedValueInBlack_ = 105;
        float kMaxRedValueInBlack_ = 130;

        float kMinGreenValueInBlack_ = 70;
        float kMaxGreenValueInBlack_ = 100;

        float kMinBlueValueInBlack_ = 50;
        float kMaxBlueValueInBlack_ = 85;


        static constexpr char kRedColor_ = 'r';
        static constexpr char kBlueColor_ = 'b';
        static constexpr char kBlackColor_ = 'n';
        static constexpr char kUndefinedColor_ = 'u';

        bool inRange(const uint8_t color, const uint8_t colorRegistered);

        bool inRangeThreshold(const double lowerBound, const double colorDetection, const double upperBound);

        void setDefaultValues();

        float kRangeTolerance_ = 20;

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

        void getRanges();
};

#endif