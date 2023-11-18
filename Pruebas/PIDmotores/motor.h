#include <Arduino.h>

#ifndef motor_h
#define motor_h

/*enum class MotorState
{
  Backward = -1,
  Stop = 0,
  Forward = 1
};*/

class motor{
    private:
        //MOTORES
        //__cpp_raw_strings motorID;
        uint8_t pwm = 0;
        int pwmPin;
        uint8_t digitalOne;
        uint8_t digitalTwo;
        //uint8_t speed = 0;
        
        //ENCODERS
        uint8_t encoderA = 0;
        uint8_t pidTics = 0;
        long long int encoderTics = 0;
    public:
        motor();//constructor
        motoresSetup(uint8_t pwmPin, uint8_t digitalOne, uint8_t digitalTwo, uint8_t encoderA);
        void initEncoder(uint8_t encoderA);
        //void setPWM(uint8_t pwm);
        /*void setDirection(uint8_t direction);
        void setSpeed(uint8_t speed);
        void stop();*/
};
#endif