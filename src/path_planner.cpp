/*
 * path_planner.cpp
 *
 *  Created on: Mar 11, 2018
 *      Author: mariaguinea
 */

#include "utils.h"


class PathPlanner {
 private:
  Vehicle ego;
  VehicleDetector detector;

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

  double getMinDistanceAhead(vector<Vehicles>cars_ahead, Vehicle &ahead);
  double getMinDistanceBehind(vector<Vehicles>cars_behind);

  bool isChangeCenterSafe();
  bool isChangeLeftSafe();
  bool isChangeRightSafe();

 public:
  PathPlanner(Vehicle ego, VehicleDetector &detector, double car_s, double prev_size) :
    ego(ego), detector(detector), car_s(car_s), prev_size(prev_size) {}

  double nextLane();
};

double PathPlanner::nextLane() {
  double next_lane;
  double current_lane = ego.getLane();
  Vehicle car_infront = detector.getCarInfront();

  if (current_lane != 1) {
    if (isChangeCenterSafe()) {
      Vehicle carNextLane = centerAhead;
      next_lane = (carNextLane.speed > car_infront.speed) ? 1.0 : current_lane;
    }
    else {
      next_lane = current_lane;
    }
  }
  else {
    bool left_change = isChangeLeftSafe();
    bool right_change = isChangeRightSafe();

    // Now decide if we move to the left or to the right
    if (left_change && !right_change) {
      next_lane = 0.0;
    }
    else if (!left_change && right_change) {
      next_lane = 2.0;
    }
    else if (left_change && right_change) {
      Vehicle car_ahead_left = leftAhead;
      Vehicle car_ahead_right = rightAhead;

      // Move to other line if the vehicles are faster than the car in front of us
      bool car_left_faster = car_ahead_left.speed>car_infront.speed ? true : false;
      bool car_right_faster = car_ahead_right.speed>car_infront.speed ? true : false;

      // Valid cars have positive id (negative id means no car)
      if (car_ahead_left.id < 0 || car_ahead_right.id < 0) {
        // If there are no cars on the left or no cars right
        // Move to lane with less traffic
        next_lane = detector.cars_left_ahead.size()<detector.cars_right_ahead.size() ? 0 : 2;
      }
      else if (car_left_faster && !car_right_faster) {
        next_lane = 0.0;
      }
      else if (!car_left_faster && car_right_faster) {
        next_lane = 2.0;
      }
      else if (car_left_faster && car_right_faster) {
        // Move to the lane with the fastest car ahead
         next_lane = car_ahead_left.speed>car_ahead_right.speed ? 0 : 2;

      }
      else if (!car_left_faster && !car_right_faster){
        // Cars on the left and right are slower. Better keep in the middle
        next_lane = 0;
      }
    }
  }

  return next_lane;
}


bool PathPlanner::isCarAhead(double min_distance_ahead) {
  if (min_distance_ahead>safety_distance) {
    printf("No cars ahead! Closest car is at %f meters\n", min_distance_ahead);
    return false;
  }
  return true;
}

bool PathPlanner::isCarBehind(double min_distance_behind) {
  if (min_distance_behind>safety_distance_behind) {
    printf("No cars behind! Closest car is at %f meters\n", min_distance_behind);
    return false;
  }
  return true;
}

double PathPlanner::getMinDistanceAhead(vector<Vehicles>cars_ahead, Vehicle &ahead) {
  double min_distance_ahead = sensor_range;
  for (auto other = cars_ahead.begin(); other!=cars_ahead.end(); ++other) {
    double other_future_s = other->s + other->getSpeed() * prev_size / points_per_second;
    double distance = abs(other_future_s-car_s);
    //double distance = abs(other->s - car_s);
    printf("CA - Gap in meters: %f\n", distance);
    if (distance<min_distance_ahead) {
      min_distance_ahead = distance;
      ahead = *other;
    }
  }
  return min_distance_ahead;
}

double PathPlanner::getMinDistanceBehind(vector<Vehicles>cars_behind) {
  double min_distance_behind = sensor_range;
  for (auto other = cars_behind.begin(); other!=cars_behind.end(); ++other) {
    double other_future_s = other->s + other->getSpeed() * prev_size / points_per_second;
    double distance = abs(other_future_s-car_s);
    //double distance = abs(other->s - car_s);
    printf("CB - Gap in meters: %f\n", distance);
    if (distance<min_distance_behind) {
      min_distance_behind = distance;
    }
  }
  return min_distance_behind;
}

bool PathPlanner::isChangeCenterSafe() {
  double min_distance_ahead = getMinDistanceAhead(detector.cars_center_ahead, centerAhead);
  double min_distance_behind = getMinDistanceBehind(detector.cars_center_behind);
  return (!isCarAhead(min_distance_ahead) && !isCarBehind(min_distance_behind));
}

bool PathPlanner::isChangeLeftSafe() {
  double min_distance_ahead = getMinDistanceAhead(detector.cars_left_ahead, leftAhead);
  double min_distance_behind = getMinDistanceBehind(detector.cars_left_behind);
  return (!isCarAhead(min_distance_ahead) && !isCarBehind(min_distance_behind));
}

bool PathPlanner::isChangeRightSafe() {
  double min_distance_ahead = getMinDistanceAhead(detector.cars_right_ahead, rightAhead);
  double min_distance_behind = getMinDistanceBehind(detector.cars_right_behind);
  return (!isCarAhead(min_distance_ahead) && !isCarBehind(min_distance_behind));
}