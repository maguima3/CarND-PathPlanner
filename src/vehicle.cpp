/*
 * vehicle.cpp
 *
 *  Created on: Mar 11, 2018
 *      Author: mariaguinea
 */

#include "vehicle.h"

Vehicle::Vehicle(double x, double y, double s, double d) :
  x(x), y(y), s(s), d(d), id(-1) {

  if (d < lane_width) { // left lane
    lane = 0.0;
  }

  else if (d>lane_width && d<2*lane_width) { // center line
    lane = 1.0;
  }

  else if (d>2*lane_width) { //right lane
    lane = 2.0;
  }
}

void Vehicle::setVelocity(double vx, double vy) {
    this->vx = vx;
    this->vy = vy;
    speed = sqrt(vx*vx + vy*vy);
  }
void Vehicle::setId(double id) {
  this->id = id;
}
double Vehicle::getSpeed() {
  return speed;
}

double Vehicle::getLane() {
  return lane;
}
