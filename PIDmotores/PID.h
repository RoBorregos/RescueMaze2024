#ifndef PID_h
#define PID_h

#include <Arduino.h>
#include "BNO.h"	
#include <math.h>


class PID {
    private:
        // TODO: Check if it is needed to add a value to the variables
        double kp{0};
        double ki{0};
        double kd{0};

        double consKp;
        double consKi;
        double consKd;

        double errorSum{0};
        double errorPrev{0};

        double maxError;
        double minOutput{30};
        double maxOutput{255};

        unsigned long timePrev;
        unsigned long sampleTime;

    public:

        PID(const double kp, const double ki, const double kd, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime);

        PID(const double kp, const double ki, const double kd);

        PID();

        void compute(const double setPoint, double &input, double &output, int &resetVariable, const double PulsesPerRev, const double countTimeSampleInSec, const bool debug=false);

        void computeRotateIzq(const double target, const double current, double &output);

        void computeRotateDer(const double target, const double current, double &output);

        void setTunings(double kp,  double ki,  double kd);

        void reset();

        void infoPID();	

        void computeStraight(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight);

};
#endif