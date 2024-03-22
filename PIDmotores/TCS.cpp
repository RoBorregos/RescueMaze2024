#include "TCS.h"

#define DEBUG_TCS 0
TCS::TCS() {
    setDefaultValues();
}

TCS::TCS(const uint8_t posMux) {
    mux.setNewChannel(posMux);
    setDefaultValues();
}

TCS::TCS(const uint8_t posMux, const int precision) {
    mux.setNewChannel(posMux);
    setDefaultValues();
    this->precision_ = precision;
}

void TCS::init() {
    mux.selectChannel();
    if (!tcs.begin()) {
        #if DEBUG_TCS
        customPrintln("No TCS34725 found ... check your connections");
        #endif
    }
}

void TCS::init(const int colors[][kColorAmount_], const int colorAmount) {
    this->colors_ = colors;
    this->colorAmount_ = colorAmount;
    init();
}

void TCS::init(const int colors[][3], const int colorAmount, const char colorList[]) {
    this->colorList_ = colorList;
    init(colors, colorAmount);
}

void TCS::init(const int colors[][3], const int colorAmount, const char colorList[], const int colorThresholds[][6]) {
    this->colorThresholds_ = colorThresholds;
    init(colors, colorAmount, colorList);
}

void TCS::setDefaultValues() {
    red_ = 0;
    green_ = 0;
    blue_ = 0;

    char tempColorList[4] = {"rnb"};
    colorList_ = tempColorList;
    colors_ = nullptr;
    colorThresholds_ = nullptr;
    precision_ = kPrecision_;
}

void TCS::updateRGB() {
    mux.selectChannel();
    tcs.setInterrupt(false);
    delay(millisToWait_);
    tcs.getRGB(&red_, &green_, &blue_);
    tcs.setInterrupt(true);
}

void TCS::updateRGBC() {
    mux.selectChannel();
    tcs.setInterrupt(false);
    delay(millisToWait_);
    uint16_t redR;
    uint16_t greenR;
    uint16_t blueR;
    uint16_t clearR;
    tcs.getRawData(&redR, &greenR, &blueR, &clearR);
    red_ = redR;
    green_ = greenR;
    blue_ = blueR;
    tcs.setInterrupt(true);
}

void TCS::printRGB() {
    updateRGBC();
    #if DEBUG_TCS
    customPrint("R:\t"); customPrint(red_);
    customPrint("\tG:\t"); customPrint(green_);
    customPrint("\tB:\t"); customPrint(blue_);
    customPrint("\n");
    #endif
}

void TCS::printRGBC() {
    double t = millis();
    updateRGBC();
    #if DEBUG_TCS
    customPrint("Time:\t"); customPrintln(millis() - t);
    customPrint("R:\t"); customPrintln(red_);
    customPrint("G:\t"); customPrintln(green_);
    customPrint("B:\t"); customPrintln(blue_);
    #endif
}

void TCS::printColor() {
    #if DEBUG_TCS
    customPrint("Color:\t");
    #endif
    const char color = (colors_) ? getColorWithPrecision() : getColor();
    #if DEBUG_TCS
    customPrintln(color);
    #endif
}

void TCS::setMux(const uint8_t posMux) {
    mux.setNewChannel(posMux);
}

void TCS::setPrecision(const uint8_t precision) {
    this->precision_ = precision;
}

char TCS::getColor() {
    updateRGB();
    char colorLetter;
    // TODO: check each color
    if (red_ < kMaxRedValueInBlue_  && green_ > kMinGreenValueInBlue_ && blue_ > kMinBlueValueInBlue_) {
        // blue
        colorLetter = 'b';
        #if DEBUG_TCS
        customPrintln("blue");
        #endif
    } else if (red_ > kMinRedValueInRed_ && green_ < kMaxGreenValueInRed_ && blue_ < kMaxBlueValueInRed_) {
        // red
        colorLetter = kRedColor_;
        #if DEBUG_TCS
        customPrintln("red");
        #endif
    } else if (red_ < kMaxRedValueInBlack_ && green_ < kMaxGreenValueInBlack_ && blue_ < kMaxBlueValueInBlack_) {
        // black
        colorLetter = kBlackColor_;
        #if DEBUG_TCS
        customPrintln("black");
        #endif
    } else {
        colorLetter = kUndifinedColor_;
        #if DEBUG_TCS
        customPrintln("unknown");
        #endif
    }
    #if DEBUG_TCS
    customPrint("colorLetter: "); customPrintln(colorLetter);
    #endif
    return colorLetter;
}

bool TCS::inRange(uint8_t colorInput, uint8_t colorRegistered) {
    return (((colorRegistered - precision_) <= colorInput) && (colorInput <= (colorRegistered + precision_)));
}

bool TCS::inRangeThreshold(double lowerBound, double colorDetection, double upperBound) {
    if (lowerBound > upperBound) {
        double temp = lowerBound;
        lowerBound = upperBound;
        upperBound = temp;
    }

    return (lowerBound <= colorDetection) && (colorDetection <= upperBound);
}

char TCS::getColorWithPrecision() {
    if (colors_ == nullptr) {
        return getColor();
    }

    updateRGBC();
    #if DEBUG_TCS
    customPrint("R:\t"); customPrintln(red);
    customPrint("G:\t"); customPrintln(green);
    customPrint("B:\t"); customPrintln(blue);
    customPrint("Precision:\t"); customPrintln(precision);
    #endif

    for (uint8_t i = 0; i < colorAmount_; ++i) {
        if (inRange(red_, colors_[i][0]) && inRange(green_, colors_[i][1]) && inRange(blue_, colors_[i][2])) {
            return colorList_[i];
        }
    }
    return kUndifinedColor_;
}

char TCS::getColorWithThresholds() {
    if (colorThresholds_ == nullptr) {
        return getColorWithPrecision();
    }
/*     customPrintln("Using thresholds");
    customPrintln("Red:\t"); customPrintln(red_);
    customPrintln("\tGreen:\t"); customPrintln(green_);
    customPrintln("\tBlue:\t"); customPrintln(blue_); */


    updateRGBC();

    for (uint8_t i = 0; i < colorAmount_; ++i) {
        if (inRangeThreshold(colorThresholds_[i][0], red_, colorThresholds_[i][1]) && inRangeThreshold(colorThresholds_[i][2], green_, colorThresholds_[i][3]) && inRangeThreshold(colorThresholds_[i][4], blue_, colorThresholds_[i][5])) {
            return colorList_[i];
        }
    }
    return kUndifinedColor_;
}

char TCS::getColorKReps(const int reps) {
    char start = getColorWithPrecision();

    if (start == kUndifinedColor_) {
        return start;
    }

    for (int i = 0; i < reps; ++i) {
        char current = getColorWithPrecision();
        if (current != start) {
            return kUndifinedColor_;
        }
    }

    return start;
}

char TCS::getColorMode(const int sampleSize, const double certainity) {
    int mode = 0;
    int unknown = sampleSize;
    int repetitions[colorAmount_];

    for (int i = 0; i < colorAmount_; ++i) {
        repetitions[i] = 0;
    }

    for (int i = 0; i < sampleSize; ++i) {
        char current = getColorWithPrecision();
        if (current == kUndifinedColor_) {
            return kUndifinedColor_;
        }

        for (int j = 0; j < colorAmount_; ++j) {
            if (current == colorList_[j]) {
                repetitions[j]++;
                break;
            }
        }
    }

    for (int i = 0; i < colorAmount_; ++i) {
        if (repetitions[mode] < repetitions[i]) {
            mode = i;
        }
        unknown -= repetitions[i];
    }
    double probability = (repetitions[mode]) / (double)sampleSize;

    if (repetitions[mode] > unknown && probability > certainity) {
        return colorList_[mode];
    }
    return kUndifinedColor_;
}

void TCS::printColorMatrix() {
    if (colors_ == nullptr) {
        #ifndef DEBUG_TCS
        customPrintln("No colors registered");
        #endif
        return;
    }

    for (int i = 0; i < colorAmount_; ++i) {
        #if DEBUG_TCS
        customPrint(colorList[i]);
        customPrint(":\t");
        customPrint(colors[i][0]);
        customPrint("\t");
        customPrint(colors[i][1]);
        customPrint("\t");
        customPrintln(colors[i][2]);
        #endif
    }
}

void TCS::printColorList() {
    if (colorList_ == nullptr) {
        #if DEBUG_TCS
        customPrintln("No colors registered");
        #endif
        return;
    }
    for (int i = 0; i < colorAmount_; ++i) {
        #if DEBUG_TCS
        customPrint(colorList[i]);
        customPrint("\t");
        #endif
    }
    #if DEBUG_TCS
    customPrintln("");
    #endif
}

