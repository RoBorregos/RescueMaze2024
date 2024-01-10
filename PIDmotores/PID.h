#ifndef PID_h
#define PID_h

#include "BNO.h"	
#include <functional>
#include <math.h>

class PID {
    private:

        double kP_{0};
        double kI_{0};
        double kD_{0};

        double errorSum_{0};
        double errorPrev_{0};

        double maxErrorSum_{0};
        double minOutput_{0};
        double maxOutput_{255};

        unsigned long timePrev_{0};
        unsigned long sampleTime_{100};

    public:

        PID(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime);

        PID(const double kP, const double kI, const double kD);

        PID();

        void computeStraight(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight);

        void computeTurn(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight, bool &clockwise);

        double computeErrorOrientation(const double targetOrientation, const double currentOrientation);
        double computeOutputModifier(const double errorOrientation, const unsigned long timeDiff);
        void compute(const double setpoint, double& input, double& output, long long& resetVariable, double (*func)(const long long, const unsigned long));
        void setTunnings(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime);
};
#endif