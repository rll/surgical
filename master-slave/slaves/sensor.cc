// sensor.cc

#include "sensor.h"

Sensor::Sensor(int c, double r, double o, int s)
{
  currentInput = 0.0;
  channel = c;
  ratio = r;
  offset = o;
  sign = s;
}

Sensor::~Sensor(void)
{
}


