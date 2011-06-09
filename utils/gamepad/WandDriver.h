#ifndef __WAND_DRIVER_H__
#define __WAND_DRIVER_H__

#include "CarDriver.h"

class WandDriver : public CarDriver
{
  public:
    WandDriver(CarTracker* ct);
    void getControls(double &c0, double &c1);

  private:
    double trackPoint(double x, double y, bool settle = true);
};

WandDriver::WandDriver(CarTracker* ct) : CarDriver(ct) {}

void WandDriver::getControls(double &c0, double &c1)
{
  double wx = theCarTracker->getWandXPos();
  double wy = theCarTracker->getWandYPos();
  c1 = theCarTracker->getWandZPos() / 1.5;
  c0 = trackPoint(wx, wy, true);
}

//Return a c0 value to track a chosen point.
double WandDriver::trackPoint(double x, double y, bool settle) {
  double xDisp = theCarTracker->getXPos()-x;
  double yDisp = theCarTracker->getYPos()-y;
  double yaw = theCarTracker->getYaw();
  double cross = xDisp*sin(yaw) - yDisp*cos(yaw);
  double dotProd = xDisp*cos(yaw) + yDisp*sin(yaw);
  double dispMag = sqrt(xDisp*xDisp + yDisp*yDisp);
  double c0 = -cross / dispMag;
  if(!settle || dotProd > 0) c0 = c0 / fabs(c0);
  return c0;
}

#endif
