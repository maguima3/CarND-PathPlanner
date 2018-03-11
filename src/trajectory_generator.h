/*
 * trajectory_generator.h
 *
 *  Created on: Mar 11, 2018
 *      Author: mariaguinea
 */

#ifndef TRAJECTORY_GENERATOR_H_
#define TRAJECTORY_GENERATOR_H_

#include "utils.h"
#include "spline.h"
#include <vector>

using namespace std;

class TrajectoryGenerator {
 private:
  // State of the car
  double car_x;
  double car_y;
  double car_yaw;
  double car_s;
  double lane;

  // Waipoints we must follow
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;

  // Create a list of evenly spaced (x,y) points, using the previously calculated waypoints plus
  // some other future points
  // Then we will calculate a line that passes through all of them
  // Using the previous points helps in obtaining a smooth trajectory
  vector<double> ptsx;
  vector<double> ptsy;

  // Reference x, y, and yaw states
  // They will reference either the starting point at where the car is,
  // or at the previous path end-point
  double ref_x;
  double ref_y;
  double ref_yaw;

  double ref_velocity;

  // Previous calculated path, not used by the simulator yet
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double prev_size; // Number of previous points

  void setReferencePoints();
  void setAnchorPoints();

 public:
  TrajectoryGenerator(double car_x, double car_y, double car_yaw, double car_s,
                      double ref_velocity, double lane,
                      vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                      vector<double> previous_path_x, vector<double> previous_path_y);

  void findTrajectory(vector<double> &next_x_vals, vector<double> &next_y_vals);

};


#endif /* TRAJECTORY_GENERATOR_H_ */
