#ifndef TCS_h
#define TCS_h

#include "Adafruit_TCS34725.h"
#include "MUX.h"
#include "CustomSerial.h"
#include <Adafruit_ADS1X15.h>
#include <Wire.h>

// TODO: check which address is the correct one
#define TCS_ADDR 0x30 

class TCS {
    private:
        Adafruit_TCS34725 tcs_ = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
        MUX mux_;
        Adafruit_ADS1115 photoresistor;

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

        float kMinRedValueInBlue_ = -6;
        float kMaxRedValueInBlue_ = 194;

        float kMinGreenValueInBlue_ = 49;
        float kMaxGreenValueInBlue_ = 249;

        float kMinBlueValueInBlue_ = 101;
        float kMaxBlueValueInBlue_ = 301;

        float kMaxRedValueInRed_ = 0;
        float kMinRedValueInRed_ = 0;

        float kMaxGreenValueInRed_ = 0;
        float kMinGreenValueInRed_ = 0;

        float kMaxBlueValueInRed_ = 0;
        float kMinBlueValueInRed_ = 0;

        float kMinRedValueInBlack_ = -37;
        float kMaxRedValueInBlack_ = 163;

        float kMinGreenValueInBlack_ = -54;
        float kMaxGreenValueInBlack_ = 146;

        float kMinBlueValueInBlack_ = -61;
        float kMaxBlueValueInBlack_ = 139;

        float kMinRedValueInCheckpoint_ = 0;
        float kMaxRedValueInCheckpoint_ = 0;

        float kMinGreenValueInCheckpoint_ = 0;
        float kMaxGreenValueInCheckpoint_ = 0;

        float kMinBlueValueInCheckpoint_ = 0;
        float kMaxBlueValueInCheckpoint_ = 0;

        float kMinRedValueInWhite_ = 0;
        float kMaxRedValueInWhite_ = 0;

        float kMinGreenValueInWhite_ = 0;
        float kMaxGreenValueInWhite_ = 0;

        float kMinBlueValueInWhite_ = 0;
        float kMaxBlueValueInWhite_ = 0;

        static constexpr char kRedColor_ = 'r';
        static constexpr char kBlueColor_ = 'B';
        static constexpr char kBlackColor_ = 'N';
        static constexpr char kCheckpointColor_ = 'C';
        static constexpr char kUndefinedColor_ = 'U';

        float kMinPhotoresistorValue_ = 11544;
        float kMaxPhotoresistorValue_ = 12544;
        static constexpr float kPhotoresistorThreshold_ = 500;

        bool inRange(const uint8_t color, const uint8_t colorRegistered);

        bool inRangeThreshold(const double lowerBound, const double colorDetection, const double upperBound);

        void setDefaultValues();

        float kRangeTolerance_ = 50;

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

        void getBlueRanges();

        void getBlackRanges();

        void getCheckpointRanges();
};

#endif