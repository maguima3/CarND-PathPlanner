/*
 * path_planner.cpp
 *
 *  Created on: Mar 11, 2018
 *      Author: mariaguinea
 */

#include "utils.h"


class PathPlanner {
 private:
  vector<vector<double>> sensorFusion;
  Vehicle ego;


 public:
  PathPlanner(Vehicle ego, vector<vector<double>> sensorFusion) :
    ego(ego), sensorFusion(sensorFusion) {}
};

// Define our car
Vehicle ego = Vehicle(car_x, car_y, car_s, car_d);
ego.setId(0);

// Vehicles around our car
vector<Vehicle> cars_left_ahead;
vector<Vehicle> cars_center_ahead;
vector<Vehicle> cars_right_ahead;

vector<Vehicle> cars_left_behind;
vector<Vehicle> cars_center_behind;
vector<Vehicle> cars_right_behind;

// Number of waypoints calculated in the last iteration, which the car didn't go through
// Would be a number close to 50 (e.g 47)
double prev_size = previous_path_x.size();

// Detect collisions
if (prev_size > 0) {
  car_s = end_path_s; // Use last point of the previous path as car_s to ease calculations
}

bool change_lane = false; // Flag

const double min_gap = 50.0; // Minimum distance between two vehicles (gap)
const double sensor_range = 100.0; // Range in which other vehicles are detected [m]
const double safety_distance = 30.0; // There's risk of collision if other cars are closer [meters]
const double safety_distance_behind = 5.0;
const double brake_distance = 30.0; // Break if there is any car closer

double closest_car_distance = 20000000; // Indicates the distance of the closest car in our lane. Controls acceleration
Vehicle car_infront;

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

  double diff = other.s - ego.s;
  bool is_ahead = (diff > 0);
  bool is_detectable = (abs(diff) <= sensor_range);

  // Calculate the lane of the other car
  if (other.d < lane_width) { // left lane
    if (is_ahead && is_detectable) {
      cars_left_ahead.push_back(other);
    }
    else if (!is_ahead && is_detectable) {
      cars_left_behind.push_back(other);
    }
  }
  else if (other.d > lane_width && other.d < 2*lane_width) { // center line
    if (is_ahead && is_detectable) {
    cars_center_ahead.push_back(other);
  }
  else if (!is_ahead && is_detectable) {
    cars_center_behind.push_back(other);
  }
  }
  else if (other.d > 2*lane_width) { //right lane
    if (is_ahead && is_detectable) {
    cars_right_ahead.push_back(other);
  }
  else if (!is_ahead && is_detectable) {
    cars_right_behind.push_back(other);
  }
  }

  // Check if the other car is in our lane
  // If it is too close, we need to change lane!
  if (other.d>lane*lane_width && other.d<(lane+1)*lane_width) {
    double car_infront_distance = abs(other.s-ego.s);
    bool is_too_close = (is_ahead) && (car_infront_distance < min_gap);

    if (is_too_close) {
      change_lane = true;
      car_infront = other;

      // Update the distance with the car in front
      if (car_infront_distance < closest_car_distance){
      closest_car_distance = car_infront_distance;
    }
    }
   }
}

// Logic about changing lanes

// Predict other cars position in the future (if we are using previous waypoints)
if (change_lane) {
  double min_distance_ahead = sensor_range;
  double min_distance_behind = sensor_range;
  bool car_ahead = true;
  bool car_behind = true;

  // Situation (1) We are on the left/right lane. We can just move to the middle
  if (lane != 1) {
    // Check vehicles in the center lane and look for a gap
    for (auto other = cars_center_ahead.begin(); other!=cars_center_ahead.end(); ++other) {
      double other_future_s = other->s + other->getSpeed() * prev_size / points_per_second;
      double distance = abs(other_future_s-car_s);
      //double distance = abs(other->s - car_s);
      printf("CA - Gap in meters: %f\n", distance);
      if (distance<min_distance_ahead) {
        min_distance_ahead = distance;
      }
    }
    for (auto other = cars_center_behind.begin(); other!=cars_center_behind.end(); ++other) {
      double other_future_s = other->s + other->getSpeed() * prev_size / points_per_second;
      double distance = abs(other_future_s-car_s);
      //double distance = abs(other->s - car_s);
      printf("CB - Gap in meters: %f\n", distance);
      if (distance<min_distance_behind) {
        min_distance_behind = distance;
      }
    }

    if (min_distance_ahead>safety_distance) {
      car_ahead = false;
      printf("No cars ahead! Closest car is at %f meters\n", min_distance_ahead);
    }
    if (min_distance_behind>safety_distance_behind) {
      printf("No cars behind! Closest car is at %f meters\n", min_distance_behind);
      car_behind = false;
    }

    if (!car_ahead && !car_behind) {
      lane = 1.0;
    }
  }

  // Situation (2) We are in the center lane. We have to decide if we move left or right
  else {
    bool left_change = false;
    bool right_change = false;
    double distance_ahead_left;
    double distance_ahead_right;
    Vehicle car_ahead_left;
    Vehicle car_ahead_right;

    // First check if we can move left
    for (vector<Vehicle>::iterator other = cars_left_ahead.begin(); other!=cars_left_ahead.end(); ++other) {
      double other_future_s = other->s + other->getSpeed() * prev_size / points_per_second;
      double distance = abs(other_future_s-car_s);
      //double distance = abs(other->s - car_s);
      printf("LA - Gap in meters: %f\n", distance);
      if (distance<min_distance_ahead) {
        min_distance_ahead = distance;
        car_ahead_left = *other;
      }
    }
    for (auto other = cars_left_behind.begin(); other!=cars_left_behind.end(); ++other) {
      double other_future_s = other->s + other->getSpeed() * prev_size / points_per_second;
      double distance = abs(other_future_s-car_s);
      //double distance = abs(other->s - car_s);
      printf("LB - Gap in meters: %f\n", distance);
      if (distance<min_distance_behind) {
        min_distance_behind = distance;
      }
    }

    if (min_distance_ahead>safety_distance) {
      car_ahead = false;
      distance_ahead_left = min_distance_ahead;
      printf("No cars ahead! Closest car is at %f meters\n", min_distance_ahead);
    }
    if (min_distance_behind>safety_distance_behind) {
      printf("No cars behind! Closest car is at %f meters\n", min_distance_behind);
      car_behind = false;
    }

    if (!car_ahead && !car_behind) {
      left_change = true;
    }

    // Then check if we can move to the right
    min_distance_ahead = sensor_range;
    min_distance_behind = sensor_range;
    car_ahead = true;
    car_behind = true;

    for (auto other = cars_right_ahead.begin(); other!=cars_right_ahead.end(); ++other) {
      double other_future_s = other->s + other->getSpeed() * prev_size / points_per_second;
      double distance = abs(other_future_s-car_s);
      //double distance = abs(other->s - car_s);
      printf("RA - Gap in meters: %f\n", distance);
      if (distance<min_distance_ahead) {
        min_distance_ahead = distance;
        car_ahead_right = *other;
      }
    }
    for (auto other = cars_right_behind.begin(); other!=cars_right_behind.end(); ++other) {
      double other_future_s = other->s + other->getSpeed() * prev_size / points_per_second;
      double distance = abs(other_future_s-car_s);
      //double distance = abs(other->s - car_s);
      printf("RB - Gap in meters: %f\n", distance);
      if (distance<min_distance_behind) {
        min_distance_behind = distance;
      }
    }

    if (min_distance_ahead>safety_distance) {
      car_ahead = false;
      distance_ahead_right = min_distance_ahead;
      printf("No cars ahead! Closest car is at %f meters\n", min_distance_ahead);
    }
    if (min_distance_behind>safety_distance_behind) {
      printf("No cars behind! Closest car is at %f meters\n", min_distance_behind);
      car_behind = false;
    }

    if (!car_ahead && !car_behind) {
      right_change = true;
    }

    // Now decide if we move to the left or to the right
    if (left_change && !right_change) {
      lane = 0;
    }
    else if (!left_change && right_change) {
      lane = 2;
    }
    else if (left_change && right_change) {
      // Move to lane with less traffic
      //size_t traffic_left = cars_left_ahead.size();
      //size_t traffic_right = cars_right_ahead.size();

      //lane = traffic_left<traffic_right ? 0 : 2;

      // Move to the lane with furtherest car ahead
      //lane = distance_ahead_left>distance_ahead_right ? 0 : 2;

      // Move to other line if the vehicles are faster than the car in front of us
      bool car_left_faster = car_ahead_left.getSpeed()>car_infront.getSpeed() ? true : false;
      bool car_right_faster = car_ahead_right.getSpeed()>car_infront.getSpeed() ? true : false;

      if (car_ahead_left.id < 0 || car_ahead_right.id < 0) {
        // If there are no cars on the left or no cars right
        // Move to lane with less traffic
        lane = cars_left_ahead.size()<cars_right_ahead.size() ? 0 : 2;
      }
      else if (car_left_faster && !car_right_faster) {
        lane = 0;
      }
      else if (!car_left_faster && car_right_faster) {
        lane = 2;
      }
      else if (car_left_faster && car_right_faster) {
        // Move to the lane with the fastest car ahead
         lane = car_ahead_left.getSpeed()>car_ahead_right.getSpeed() ? 0 : 2;

      }
      else if (!car_left_faster && !car_right_faster){
        // Cars on the left and right are slower. Better keep in the middle
        lane = 0;
      }

    }
  }
}

// Reduce target velocity if we are too close to other cars
if (closest_car_distance < brake_distance) {

if (ref_velocity > car_infront.getSpeed()) {
  ref_velocity -= .2; // m/s

} else {
  ref_velocity += .2; // m/s
}

} else if (ref_velocity < max_velocity){
ref_velocity += .6;
}
