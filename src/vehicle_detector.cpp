/*
 * vehicle_detector.cpp
 *
 *  Created on: Mar 11, 2018
 *      Author: mariaguinea
 */

#include "vehicle_detector.h"

VehicleDetector::VehicleDetector(Vehicle &ego, double lane, vector<vector<double>> sensor_fusion) {

  this->ego = ego;
  this->lane = lane;
  this->infront_tooClose = false;
  this->closest_car_distance = 2000.0;

  // Check where are the other cars in the road
  for (int i=0; i<sensor_fusion.size(); ++i) {
    double other_id = sensor_fusion[i][0];
    double other_x = sensor_fusion[i][1];
    double other_y = sensor_fusion[i][2];
    double other_vx = sensor_fusion[i][3];
    double other_vy = sensor_fusion[i][4];
    double other_s = sensor_fusion[i][5];
    double other_d = sensor_fusion[i][6];

    Vehicle other = Vehicle(other_x, other_y, other_s, other_d);
    other.setVelocity(other_vx, other_vy);
    other.setId(other_id);


    double distance = other.getS() - ego.getS();
    bool is_ahead = (distance > 0);
    bool is_detectable = (abs(distance) <= sensor_range);

    // Calculate the lane of the other car
    if (other.getLane() == 0.0) { // left lane
      if (is_ahead && is_detectable) {
        cars_left_ahead.push_back(other);
      }
      else if (!is_ahead && is_detectable) {
        cars_left_behind.push_back(other);
      }
    }

    else if (other.getLane() == 1.0) { // center lane
      if (is_ahead && is_detectable) {
        cars_center_ahead.push_back(other);
      }
      else if (!is_ahead && is_detectable) {
        cars_center_behind.push_back(other);
      }
    }

    else if (other.getLane() == 2.0) { // right lane
      if (is_ahead && is_detectable) {
        cars_right_ahead.push_back(other);
      }
      else if (!is_ahead && is_detectable) {
        cars_right_behind.push_back(other);
      }
    }

    // Check if the other car is in our lane
    // If it is too close, we will need to change lane
    if (other.getLane() == lane) {
      double distance_ahead = abs(distance);
      if ((is_ahead) && (distance_ahead < min_gap)) {
        printf("Vehicle too close! Distance %f\n", distance_ahead);
        infront_tooClose = true;
        car_infront = other;

        // Update the closest_car_distance
        if (distance_ahead < closest_car_distance){
          closest_car_distance = distance_ahead;

        }
      }
    }
  }
}

bool VehicleDetector::getCarInfront(Vehicle &infront) {
  if (infront_tooClose) {
    infront = car_infront;
    return true;
  }
  else {
    return false;
  }
}



