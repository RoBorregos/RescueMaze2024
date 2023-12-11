#ifndef PID_h
#define PID_h

#include "BNO.h"	
//#include "Encoder.h"
#include <math.h>


class PID {
    private:
        // TODO: Check if it is needed to add a value to the variables
        double kP{0};
        double kI{0};
        double kD{0};

        double conskP;
        double conskI;
        double conskD;

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

        void compute(const double setPoint, double &input, double &output, int &resetVariable, const double PulsesPerRev, const double countTimeSampleInSec, const bool debug=false);

        void computeRotateLeft(const double target, const double current, double &output);

        void computeRotateRight(const double target, const double current, double &output);

        void setTunings(double kP,  double kI,  double kD);

        void reset();

        void infoPID();	

        void computeStraight(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight);

        void  computeSpeedPerMotor(double targetSpeed, double &currentSpeed, uint8_t tics, double &output);

        void computeStraightGiro(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight);

};
#endif