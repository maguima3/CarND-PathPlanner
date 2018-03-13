/*
 * path_planner.cpp
 *
 *  Created on: Mar 11, 2018
 *      Author: mariaguinea
 */
#include "path_planner.h"


PathPlanner::PathPlanner(double lane, VehicleDetector &detector, double car_s, double prev_size) :
    lane(lane), detector(detector), car_s(car_s), prev_size(prev_size) {}

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

double PathPlanner::getMinDistanceAhead(vector<Vehicle>cars_ahead, Vehicle &ahead) {
  double min_distance_ahead = sensor_range;
  for (auto other = cars_ahead.begin(); other!=cars_ahead.end(); ++other) {
    double other_future_s = other->getS() + other->getSpeed() * prev_size / points_per_second;
    double distance = abs(other_future_s-car_s);
//    printf("Ahead - %f\t", distance);
    if (distance<min_distance_ahead) {
      min_distance_ahead = distance;
      ahead = *other;
    }
  }
  return min_distance_ahead;
}

double PathPlanner::getMinDistanceBehind(vector<Vehicle>cars_behind) {
  double min_distance_behind = sensor_range;
  for (auto other = cars_behind.begin(); other!=cars_behind.end(); ++other) {
    double other_future_s = other->getS() + other->getSpeed() * prev_size / points_per_second;
    double distance = abs(other_future_s-car_s);
//    printf("\nBehind - %f\t", distance);
    if (distance<min_distance_behind) {
      min_distance_behind = distance;
    }
  }
  return min_distance_behind;
}

bool PathPlanner::isChangeCenterSafe() {
  printf("Center?\n");
  double min_distance_ahead = getMinDistanceAhead(detector.cars_center_ahead, centerAhead);
  double min_distance_behind = getMinDistanceBehind(detector.cars_center_behind);
  return (!isCarAhead(min_distance_ahead) && !isCarBehind(min_distance_behind));
}

bool PathPlanner::isChangeLeftSafe() {
  printf("Left?\n");
  double min_distance_ahead = getMinDistanceAhead(detector.cars_left_ahead, leftAhead);
  double min_distance_behind = getMinDistanceBehind(detector.cars_left_behind);
  return (!isCarAhead(min_distance_ahead) && !isCarBehind(min_distance_behind));
}

bool PathPlanner::isChangeRightSafe() {
  printf("Right?\n");
  double min_distance_ahead = getMinDistanceAhead(detector.cars_right_ahead, rightAhead);
  double min_distance_behind = getMinDistanceBehind(detector.cars_right_behind);
  return (!isCarAhead(min_distance_ahead) && !isCarBehind(min_distance_behind));
}


double PathPlanner::nextLane() {

  double current_lane = lane; //ego.getLane()
  double next_lane = current_lane;

  Vehicle car_infront;
  detector.getCarInfront(car_infront);

  // Current lane is left or right
  if (current_lane != 1) {
    if (isChangeCenterSafe()) {
      // If there is no car in the center, change to the center lane
      // Valid cars have positive id (negative id means no car)
      if (centerAhead.getId() < 0.0) {
        next_lane = 1.0;
      }
      else {
        // Move to the center lane if it moves faster or a little bit slower (speed_buffer m/s slower)
        // It is better to move to the center, as it offers more possibilities
        //printf("Speed of the car in the center: %f\n", centerAhead.getSpeed());
        //printf("Speed of the car in front: %f\n", car_infront.getSpeed());
        printf("Cheking if the car in the center is faster...\n");
        next_lane = (centerAhead.getSpeed()+speedBuffer>car_infront.getSpeed()) ? 1.0 : current_lane;
      }
    }
  }
  // Center lane
  else {
    // Need to check if left/right are safe first, to get correct ids!!
    bool left_change = isChangeLeftSafe();
    bool right_change = isChangeRightSafe();

    Vehicle car_ahead_left = leftAhead;
    Vehicle car_ahead_right = rightAhead;

    if (left_change || right_change) {
      // If one lane is empty ahead, move to it
      // Valid cars have positive id (negative id means no car)
      // Prioritize moving left (normal traffic rule)
      if ((left_change && car_ahead_left.getId() < 0.0)) {
        next_lane = 0.0;
      }
      else if ((right_change && car_ahead_right.getId() < 0.0)) {
        next_lane = 2.0;
      }

      // Check the space ahead in the other lines
      double left_future_s = leftAhead.getS() + leftAhead.getSpeed() * prev_size / points_per_second;
      double distance_left = abs(left_future_s-car_s);
      double right_future_s = rightAhead.getS() + rightAhead.getSpeed() * prev_size / points_per_second;
      double distance_right = abs(right_future_s-car_s);

      bool bigSpaceLeft = distance_left>3*safety_distance;
      bool bigSpaceRight = distance_right>3*safety_distance;

      // Check if the other lanes are faster
      // Prioritize to stay in the center
      bool car_left_faster = (car_ahead_left.getSpeed()>car_infront.getSpeed()+speedBuffer) ? true : false;
      bool car_right_faster = (car_ahead_right.getSpeed()>car_infront.getSpeed()+speedBuffer) ? true : false;

      // Move to the lane with more space ahead
      if (left_change && bigSpaceLeft) {
        next_lane = 0.0;
      }
      else if (right_change && bigSpaceRight) {
        next_lane = 2.0;
      }
      // Move to the lane with the fastest car ahead
      else if (left_change && car_left_faster && right_change && car_right_faster) {
         next_lane = (car_ahead_left.getSpeed()>car_ahead_right.getSpeed()) ? 0 : 2;
      }
      else if (left_change && car_left_faster) {
        next_lane = 0.0;
      }
      else if (right_change && car_right_faster) {
        next_lane = 2.0;
      }
      else if (!car_left_faster && !car_right_faster){
        // Cars on the left and right are slower. Better keep in the middle
        next_lane = current_lane;
      }
    }
  }
  printf("NEXT LANE: %d\n", (int)next_lane);
  return next_lane;
}
