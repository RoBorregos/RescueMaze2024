#ifndef PID_h
#define PID_h

#include "BNO.h"
#include <math.h>

class PID {
    private:

        double kP_{0.0};
        double kI_{0.0};
        double kD_{0.0};

        double errorSum_{0};
        double errorPrev_{0};

        double kMaxErrorSum_{4000};
        double kMinOutput_{0};
        double kMaxOutput_{255};

        unsigned long timePrev_{0};
        unsigned long kSampleTime_{100};

        double kBaseModifier_{0};
        
        double kMaxOrientationError_{0.8};

    public:

        PID(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime, const double baseModifier_, const double kMaxOrientationError_);

        PID(const double kP, const double kI, const double kD);

        PID();

        void computeStraight(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight);

        void computeStraightReset(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight);

        void computeTurn(const double targetOrientation, const double currentOrientation, double &speed, bool &clockwise);

        double computeErrorOrientation(const double targetOrientation, const double currentOrientation);
        double computeOutputModifier(const double errorOrientation, const unsigned long timeDiff);
        void compute(const double setpoint, double& input, double& output, long long& resetVariable, double (*func)(const long long, const unsigned long));
        void setTunningsMotors(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime);
        void setTunnings(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime, const double baseModifier, const double kMaxOrientationError);
        void setBaseSpeed(const double baseModifier);
};
#endif