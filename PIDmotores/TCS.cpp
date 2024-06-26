#include "TCS.h"

#define DEBUG_TCS 1
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

void TCS::init(const int16_t colors[][3], const int8_t colorAmount, const char colorList[], const int16_t colorThresholds[][6]) {
    this->colorThresholds_ = colorThresholds;
    this->colorList_ = colorList;
    this->colors_ = colors;
    this->colorAmount_ = colorAmount;
    init();
}

void TCS::setDefaultValues() {
    red_ = 0;
    green_ = 0;
    blue_ = 0;

    const char tempColorList[4] = {"rnb"};
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
    uint16_t redR;
    uint16_t greenR;
    uint16_t blueR;
    uint16_t clearR;
    mux.selectChannel();
    tcs.setInterrupt(false);
    delay(millisToWait_);
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
    const unsigned long t = millis();
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
        colorLetter = kBlueColor_;
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
        colorLetter = kUndefinedColor_;
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
        const double temp = lowerBound;
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
    customPrint("R:\t"); customPrintln(red_);
    customPrint("G:\t"); customPrintln(green_);
    customPrint("B:\t"); customPrintln(blue_);
    customPrint("Precision:\t"); customPrintln(precision_);
    #endif

    for (uint8_t i = 0; i < colorAmount_; ++i) {
        if (inRange(red_, colors_[i][0]) &&
            inRange(green_, colors_[i][1]) &&
            inRange(blue_, colors_[i][2])) {

            return colorList_[i];
        }
    }

    return kUndefinedColor_;
}

char TCS::getColorWithThresholds() {
    if (colorThresholds_ == nullptr) {
        return getColorWithPrecision();
    }
    updateRGBC();

    for (uint8_t i = 0; i < colorAmount_; ++i) {
        if (inRangeThreshold(colorThresholds_[i][0], red_, colorThresholds_[i][1]) &&
            inRangeThreshold(colorThresholds_[i][2], green_, colorThresholds_[i][3]) &&
            inRangeThreshold(colorThresholds_[i][4], blue_, colorThresholds_[i][5])) {

            return colorList_[i];
        }
    }

    return kUndefinedColor_;
}

char TCS::getColorKReps(const int reps) {
    const char start = getColorWithPrecision();

    if (start == kUndefinedColor_) {
        return start;
    }

    for (uint8_t i = 0; i < reps; ++i) {
        const char current = getColorWithPrecision();
        if (current != start) {
            return kUndefinedColor_;
        }
    }

    return start;
}

char TCS::getColorMode(const int sampleSize, const double certainity) {
    uint8_t mode = 0;
    uint16_t unknown = sampleSize;
    uint8_t repetitions[colorAmount_];

    for (uint8_t i = 0; i < colorAmount_; ++i) {
        repetitions[i] = 0;
    }

    for (uint8_t i = 0; i < sampleSize; ++i) {
        const char current = getColorWithPrecision();
        if (current == kUndefinedColor_) {
            return kUndefinedColor_;
        }

        for (uint8_t j = 0; j < colorAmount_; ++j) {
            if (current == colorList_[j]) {
                repetitions[j]++;
                break;
            }
        }
    }

    for (uint8_t i = 0; i < colorAmount_; ++i) {
        if (repetitions[mode] < repetitions[i]) {
            mode = i;
        }
        unknown -= repetitions[i];
    }
    const double probability = (repetitions[mode]) / (double)sampleSize;

    if (repetitions[mode] > unknown && probability > certainity) {
        return colorList_[mode];
    }
    return kUndefinedColor_;
}

void TCS::printColorMatrix() {
    #if DEBUG_TCS
    if (colors_ == nullptr) {
        customPrintln("No colors registered");
        return;
    }

    for (uint8_t i = 0; i < colorAmount_; ++i) {
        customPrint(colorList_[i]);
        customPrint(":\t");
        customPrint(colors_[i][0]);
        customPrint("\t");
        customPrint(colors_[i][1]);
        customPrint("\t");
        customPrintln(colors_[i][2]);
    }
    #endif
}

void TCS::printColorList() {
#if DEBUG_TCS
    if (colorList_ == nullptr) {
        customPrintln("No colors registered");
        return;
    }

    for (uint8_t i = 0; i < colorAmount_; ++i) {
        customPrint(colorList_[i]);
        customPrint("\t");
    }

    customPrintln("");
#endif
}

