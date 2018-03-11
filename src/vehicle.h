/*
 * vehicle.h
 *
 *  Created on: Mar 11, 2018
 *      Author: mariaguinea
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "utils.h"

class Vehicle {
 private:
 public:
  double id;
  double x;
  double y;
  double s;
  double d;
  double vx;
  double vy;
  double yaw;
  double lane;
  double speed;

  Vehicle(double x=0, double y=0, double s=0, double d=0);

  void setVelocity(double vx, double vy);
  void setId(double id);
  double getSpeed();
  double getLane();
};


#endif /* VEHICLE_H_ */
