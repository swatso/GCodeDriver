#pragma once

#include <Arduino.h>

namespace gpio {

void begin();
int stepSize();
int speed();
int direction();

void speedHelper(); 
void bearingEncoderHelper();

}  // namespace gpio
