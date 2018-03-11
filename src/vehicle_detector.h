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
  double lane;

  Vehicle car_infront;

 public:
  VehicleDetector(Vehicle ego, double lane, vector<vector<double>> sensor_fusion);

  bool static infront_tooClose;
  double static closest_car_distance;

  // Vehicles around ego car
  vector<Vehicle> cars_left_ahead;
  vector<Vehicle> cars_center_ahead;
  vector<Vehicle> cars_right_ahead;

  vector<Vehicle> cars_left_behind;
  vector<Vehicle> cars_center_behind;
  vector<Vehicle> cars_right_behind;

  bool getCarInfront(Vehicle &infront);

};





#endif /* VEHICLE_DETECTOR_H_ */
