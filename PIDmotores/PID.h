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
        double errorSumVlx_{0};
        double errorPrev_{0};
        double errorPrevVlx_{0};

        double kMaxErrorSum_{4000};
        double kMinOutput_{0};
        double kMaxOutput_{255};

        unsigned long timePrev_{0};
        unsigned long kSampleTime_{100};

        double errorVLXLeftPrev = 0;
        double errorVLXRightPrev = 0;
        double errorOrientationPrev = 0;

        double kBaseModifier_{0};
        
        double kMaxError_{0.8};

        double kMaxErrorVlx_{1.0};


        const double kWeightVlx = 0.6;

        const double kWeightBNO = 0.4;

    public:

        PID(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime, const double baseModifier_, const double kMaxOrientationError_);

        PID(const double kP, const double kI, const double kD);

        PID();

        void computeStraight(const double targetOrientation, const double currentOrientation, double &outputLeft, double &outputRight);

        void computeTurn(const double targetOrientation, const double currentOrientation, double &speed, bool &clockwise);

        void computeDistance(const double targetDistance, const double currentDistance, double &outputLeft, double &outputRight);

        void computeDistance(const double setpoint, const double input1, const double input2, double& outputLeft, double& outputRight, double currentOrientation, double setpointOrientation);

        double computeErrorOrientation(const double targetOrientation, const double currentOrientation);
        double computeOutputModifier(const double errorOrientation, const unsigned long timeDiff);
        void compute(const double setpoint, double& input, double& output, long long& resetVariable, double (*func)(const long long, const unsigned long));
        void setTunnings(const double kP, const double kI, const double kD, const double minOutput, const double maxOutput, const double maxErrorSum, const long sampleTime, const double baseModifier, const double kMaxError);
        void setBaseSpeed(const double baseModifier);
        void resetPID();
};
#endif