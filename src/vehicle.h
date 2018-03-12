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

 public:
  Vehicle(double x=0, double y=0, double s=0, double d=0);

  void setVelocity(double vx, double vy);
  void setId(double id);
  double getId();
  double getSpeed();
  void setSpeed(double speed);
  double getLane();
  void setYaw(double yaw);
  double getS();

};


#endif /* VEHICLE_H_ */
