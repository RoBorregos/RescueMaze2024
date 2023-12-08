#ifndef Movement_h
#define Movement_h

#include "Arduino.h"
#include "Motor.h"
#include "BNO.h"



class Movement{
    private:


        Motor motorFL;
        Motor motorFR;
        Motor motorBL;
        Motor motorBR;
        /*float RMPFL, RMPFR, RMPBL, RMPBR;
        int FLtics, FRtics, BLtics, BRtics;
        int FLticsViejos, FRticsViejos, BLticsViejos, BRticsViejos;*/
        unsigned long next_time;
        // float kp, ki, kd;
        /*float pwmInicialFL,errorPrevFL;
        float pwmInicialFR,errorPrevFR;
        float pwmInicialBL,errorPrevBL;
        float pwmInicialBR,errorPrevBR;*/
        float errorPrevOrientation;
        float errorAcumuladoOrientation;

        long long int ticsCounter=0;
        int pidTics = 0;

        double currentSpeed = 0;
        double targetSpeed = 0;

        Motor motor[4];

    public:
        Movement();

        void setup();
        void moveForward(int pwmA, int pwmB, int pwmC, int pwmD);
        void updateRPM();
        void setSpeed(float targetSpeed,float orientation,BNO bno);
        //getters
        Motor getMotorFL();
        Motor getMotorFR();
        Motor getMotorBL();
        Motor getMotorBR();
        float getRPMFL();
        float getRPMFR();
        float getRPMBL();
        float getRPMBR();

        float getPWMInicialFL();
        float getPWMInicialFR();
        float getPWMInicialBL();
        float getPWMInicialBR();
        void setupInternal(MotorID motorId);

        void PIDRotateTunings(double kp, double ki, double kd);
        void PIDStraightTunings(double kp, double ki, double kd);
        void motorRotateDerPID(double targetAngle, double currentAngle);
        void motorRotateIzqPID(double targetAngle, double currentAngle);
        void motorSpeedPID(double targetSpeed, bool debug=false);
        void motorSpeedPWM(double targetSpeed_);

        void stopMotors();
        void forwardMotors(uint8_t pwms[4]);
        void backwardMotors(uint8_t pwms[4]);

        void setSpeed(double targetSpeed);

        void moveMotors(MotorState state);


        void updateTics(MotorID motorId);

        int getBackLeftEncoderTics();
        int getBackRightEncoderTics();
        int getFrontLeftEncoderTics();
        int getFrontRightEncoderTics();

        void setSpeed(MotorID motorId, double speed);

        void setMotorSpeed(int leftSpeed, int rightSpeed);
        
        /*void moveBackward();
        void turnLeft();
        void turnRight();
        void stop();*/
};

#endif