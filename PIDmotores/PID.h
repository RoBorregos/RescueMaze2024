#ifndef PID_h
#define PID_h

#include "BNO.h"	
//#include "Encoder.h"
#include <math.h>

class PID {
    private:
        double kP_{0};
        double kI_{0};
        double kD_{0};

        double errorSum{0};
        double errorPrev{0};

        double maxError;
        double minOutput{30};
        double maxOutput{255};

        unsigned long timePrev;
        unsigned long sampleTime;

    public:

        PID(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime);

        PID(const double kP, const double kI, const double kD);

        PID();

        void computeStraight(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight);

        void computeTurn(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight, bool &clockwise);

        double computeErrorOrientation(const double targetOrientation, const double currentOrientation);
        double computeOutputModifier(const double errorOrientation, const unsigned long timeDiff);

        void compute(const double setpoint, double &input, double &output, int &resetVariable, const double pulsesPerRev, const double countTimeSampleInSec);

};
#endif