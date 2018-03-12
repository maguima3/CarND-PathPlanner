/*
 * path_planner.h
 *
 *  Created on: Mar 11, 2018
 *      Author: mariaguinea
 */

#ifndef PATH_PLANNER_H_
#define PATH_PLANNER_H_

#include "vehicle.h"
#include "vehicle_detector.h"
#include <vector>
using namespace std;

class PathPlanner {
 private:
//  Vehicle ego;
  VehicleDetector detector;

  double lane;

  // Ego car s value in the future (if there are points unused in the previous path)
  // If there is not previous path, then it is the current s position of ego
  double car_s;

  // Number of waypoints calculated in the last iteration, which the car didn't go through
  // Would be a number close to 50 (e.g 47)
  double prev_size;

  Vehicle centerAhead;
  Vehicle leftAhead;
  Vehicle rightAhead;

  bool isCarAhead(double min_distance_ahead);
  bool isCarBehind(double min_distance_behind);

  double getMinDistanceAhead(vector<Vehicle>cars_ahead, Vehicle &ahead);
  double getMinDistanceBehind(vector<Vehicle>cars_behind);

  bool isChangeCenterSafe();
  bool isChangeLeftSafe();
  bool isChangeRightSafe();

  double speedBuffer = 0.5;

 public:

  PathPlanner(double lane, VehicleDetector &detector, double car_s, double prev_size);
  double nextLane();
};



#endif /* PATH_PLANNER_H_ */
