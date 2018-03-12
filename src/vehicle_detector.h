/*
 * vehicle_detector.h
 *
 *  Created on: Mar 11, 2018
 *      Author: mariaguinea
 */

#ifndef VEHICLE_DETECTOR_H_
#define VEHICLE_DETECTOR_H_

#include "vehicle.h"
#include <vector>
using namespace std;

class VehicleDetector {
 private:
  Vehicle ego;
  double lane; // Using ego.getLane() lead to unstability when changing lanes
  Vehicle car_infront;

 public:
  VehicleDetector(Vehicle &ego, double lane, vector<vector<double>> sensor_fusion);

  // Flag to determine if a car is too close
  bool infront_tooClose;
  // Distance to the closest car ahead
  double closest_car_distance;

  // Vehicles around ego car which are "detectable" (distance to them is less than sensor_range)
  vector<Vehicle> cars_left_ahead;
  vector<Vehicle> cars_center_ahead;
  vector<Vehicle> cars_right_ahead;
  vector<Vehicle> cars_left_behind;
  vector<Vehicle> cars_center_behind;
  vector<Vehicle> cars_right_behind;

  // Returns the car that is closest to ego, in front of our lane
  // If true, the car in front is given back
  bool getCarInfront(Vehicle &infront);

};





#endif /* VEHICLE_DETECTOR_H_ */
