#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>
#include "Motores.h"


namespace Encoder{
  void updateTics(Motor *motor);

  void backLeftEncoder();
  void frontLeftEncoder();
  void backRightEncoder();
  void frontRightEncoder();
  
};

#endif 