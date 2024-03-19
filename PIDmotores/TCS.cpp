#include "TCS.h"

#define DEBUG_TCS 0
TCS::TCS() {
    setDefaultValues();
}

TCS::TCS(uint8_t posMux) {
    mux.setPos(posMux);
    setDefaultValues();
}

TCS::TCS(uint8_t posMux, uint8_t precision) {
    mux.setPos(posMux);
    setDefaultValues();
    this->precision = precision;
}

void TCS::init() {
    mux.selectChannel();
    if (!tcs.begin()) {
        customPrintln("No TCS34725 found ... check your connections");
    }
}

void TCS::init(const uint8_t colors[][3], const uint8_t colorAmount) {
    this->colors = colors;
    this->colorAmount = colorAmount;
    init();
}

void TCS::init(const char colors[][], const uint8_t colorAmount, const char colorList[]) {
    this->colorList = colorList;
    init(colors, colorAmount);
}

void TCS::init(const char colors[][], const uint8_t colorAmount, const char colorList[], const int colorThresholds[][6]) {
    this->colorThresholds = colorThresholds;
    init(colors, colorAmount, colorList);
}

void TCS::setDefaultValues() {
    red = 0;
    green = 0;
    blue = 0;

    char tempColorList[4] = {"rnb"};
    colorList = tempColorList;
    colors = nullptr;
    colorThresholds = nullptr;
    precision = 10;
}

void TCS::updateRGB() {
    mux.selectChannel();
    tcs.setInterrupt(false);
    delay(50);
    tcs.getRGB(&red, &green, &blue);
    tcs.setInterrupt(true);
}

void TCS::updateRGBC() {
    mux.selectChannel();
    tcs.setInterrupt(false);
    delay(50);
    uint16_t red_r, green_r, blue_r, clear_r;
    tcs.getRawData(&red_r, &green_r, &blue_r, &clear_r);
    red = red_r;
    green = green_r;
    blue = blue_r;
    tcs.setInterrupt(true);
}

void TCS::printRGB() {
    updateRGBC();
    customPrint("R:\t"); customPrintln(red);
    customPrint("G:\t"); customPrintln(green);
    customPrint("B:\t"); customPrintln(blue);
}

void TCS::printRGBC() {
    double t = millis();
    updateRGBC();

    customPrint("Time:\t"); customPrintln(millis() - t);
    customPrint("R:\t"); customPrintln(red);
    customPrint("G:\t"); customPrintln(green);
    customPrint("B:\t"); customPrintln(blue);
}

void TCS::printColor() {
    customPrint("Color:\t");
    char color = (colors) ? getColorWithPrecision() : getColor();
    customPrintln(color);
}

void TCS::setMux(uint8_t posMux) {
    mux.setPos(posMux);
}

void TCS::setPrecision(uint8_t precision) {
    this->precision = precision;
}

char TCS::getColor() {
    updateRGB();
    char colorLetter;
    // TODO: check each color
    if (red < 110  && green > 80 && blue > 70) {
        // blue
        colorLetter = 'b';
    } else if (red > 110 && green < 80 && blue < 70) {
        // red
        colorLetter = 'r';
    } else if (red < 110 && green < 80 && blue < 70) {
        // black
        colorLetter = 'n';
    }
    return colorLetter;
}

bool TCS::inRange(uint8_t colorInput, uint8_t colorRegistered) {
    return (((registeredColor - precision) <= colorInput) && (colorInput <= (registeredColor + precision)));
}

bool TCS::inRangeThreshold(double lowerBound, double colorDetection, double upperBound) {
    if (lowerBound > upperBound) {
        double temp = lowerBound;
        lowerBound = upperBound;
        upperBound = temp;
    }

    return (lowerBound <= colorDetection) || (colorDetection <= upperBound);
}

char TCS::getColorWithPrecision() {
    if (colors == nullptr) {
        return getColor();
    }

    updateRGBC();

    customPrint("R:\t"); customPrintln(red);
    customPrint("G:\t"); customPrintln(green);
    customPrint("B:\t"); customPrintln(blue);
    customPrint("Precision:\t"); customPrintln(precision);

    for (uint8_t i = 0; i < colorAmount; ++i) {
        if (inRange(red, colors[i][0]) && inRange(green, colors[i][1]) && inRange(blue, colors[i][2])) {
            return colorList[i];
        }
    }
    return 'u';
}

char TCS::getColorWithThresholds() {
    if (colorThresholds == nullptr) {
        return getColorWithPrecision();
    }

    updateRGBC();

    for (uint8_t i = 0; i < colorAmount; ++i) {
        if (inRangeThreshold(colorThresholds[i][0], red, colorThresholds[i][1]) && inRangeThreshold(colorThresholds[i][2], green, colorThresholds[i][3]) && inRangeThreshold(colorThresholds[i][4], blue, colorThresholds[i][5])) {
            return colorList[i];
        }
    }
    return 'u';
}

char TCS::getColorKReps(int reps) {
    char start = getColorWithPrecision();

    if (start == 'u') {
        return start;
    }

    for (int i = 0; i < reps; ++i) {
        char current = getColorWithPrecision();
        if (current != start) {
            return 'u';
        }
    }

    return start;
}

char TCS::getColorMode(int sampleSize, double certainity) {
    int repetitions[colors];

    for (int i = 0; i < colors; ++i) {
        repetitions[i] = 0;
    }

    for (int i = 0; i < sampleSize; ++i) {
        char current = getColorWithPrecision();
        if (current == 'u') {
            return 'u';
        }
        for (int j = 0; j < colors; ++j) {
            if (current == colorList[j]) {
                repetitions[j]++;
                break;
            }
        }
    }

    int mode = 0, unknown = sampleSize;

    for (int i = 0; i < colors; ++i) {
        if (repetitions[mode] < repetitions[i]) {
            mode = i;
        }
        unknown -= repetitions[i];
    }
    double probability = (repetitions[mode]) / (double)sampleSize;

    if (repetitions[mode] > unknown && probability > certainity) {
        return colorList[mode];
    }
    return 'u';
}
void TCS::printColorMatrix() {
    if (colors == nullptr) {
        customPrintln("No colors registered");
        return;
    }
    for (int i = 0; i < colors; ++i) {
        customPrint(colorList[i]);
        customPrint(":\t");
        customPrint(colors[i][0]);
        customPrint("\t");
        customPrint(colors[i][1]);
        customPrint("\t");
        customPrintln(colors[i][2]);
    }
}

void TCS::printColorList() {
    if (colorList == nullptr) {
        customPrintln("No colors registered");
        return;
    }
    for (int i = 0; i < colors; ++i) {
        customPrint(colorList[i]);
        customPrint("\t");
    }
    customPrintln("");
}

