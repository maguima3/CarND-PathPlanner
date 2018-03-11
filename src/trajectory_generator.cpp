/*
* trajectory_generator.cpp
*
*  Created on: Mar 11, 2018
*      Author: mariaguinea
*/

#include "trajectory_generator.h"


TrajectoryGenerator::TrajectoryGenerator(double car_x, double car_y, double car_yaw, double car_s,
                    double ref_velocity, double lane,
                    vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
                    vector<double> previous_path_x, vector<double> previous_path_y) :
    car_x(car_x), car_y(car_y), car_yaw(car_yaw), car_s(car_s),
    ref_velocity(ref_velocity), lane(lane),
    map_waypoints_x(map_waypoints_x), map_waypoints_y(map_waypoints_y), map_waypoints_s(map_waypoints_s),
    previous_path_x(previous_path_x), previous_path_y(previous_path_y)
{
  prev_size = previous_path_x.size();
}

void TrajectoryGenerator::setReferencePoints() {
  // First, calculate two points that are tangent to the current trajectory of the car
  if (prev_size < 2) {
    // If the previous points are almost empty (because the program has just started), use the cars position
    ref_x = car_x;
    ref_y = car_y;
    ref_yaw = car_yaw;

    double ref_x_prev = ref_x - cos(ref_yaw);
    double ref_y_prev = ref_y - sin(ref_yaw);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  else {
    // Use the previous path's end point as starting reference
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];

    ref_yaw = atan2(ref_y - ref_y_prev,
                    ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
}

void TrajectoryGenerator::setAnchorPoints() {
  // Second, add three "anchor" points, located at 30, 60 and 90 meters from car_s
  // This helps to find a smooth trajectory
  vector<double> next_wp0 = getXY(car_s+ 30, (lane*lane_width + lane_width/2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+ 60, (lane*lane_width + lane_width/2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+ 90, (lane*lane_width + lane_width/2), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
}

void TrajectoryGenerator::findTrajectory(vector<double> &next_x_vals, vector<double> &next_y_vals) {

  setReferencePoints();
  setAnchorPoints();


  // To help spline find a trajectory, we translate the coordinate system
  // So that the ref_x, ref_y is at (0, 0), and its ref_yaw is 0 degrees
  for (int i=0; i<ptsx.size(); ++i) {
    // Shift
    double shiftx = ptsx[i] - ref_x;
    double shifty = ptsy[i] - ref_y;
    // Rotation
    ptsx[i] = (shiftx*cos(0-ref_yaw) - shifty*sin(0-ref_yaw));
    ptsy[i] = (shiftx*sin(0-ref_yaw) + shifty*cos(0-ref_yaw));
  }

  // Create a spline
  tk::spline s;
  s.set_points(ptsx, ptsy);

  // Add the previous calculated waypoints to the next trajectory
  // Adding previous path's points helps with transitions
  for (int i=0; i<prev_size; ++i) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  // After the previous waypoints, we have to add more points till we have "50" horizon points
  // Calculate how to break up spline points to go to the desired velocity
  double target_x = 30;
  double target_y = s(target_x);
  double target_distance = sqrt((target_x*target_x + target_y*target_y));

  // Fill in the rest of the path planner with points extrapolated from the spline
  double x_add_on = 0;
  for (int i=0; i<=horizon-prev_size; ++i) {
    double step = ref_velocity / points_per_second;
    double N = target_distance / step; // Number of "slots"
    double x_next_new = x_add_on + (target_x/N); // Take one of those steps
    double y_next_new = s(x_next_new);

    x_add_on = x_next_new;

    // Undo the previous translation
    double curr_x_ref = x_next_new;
    double curr_y_ref = y_next_new;

    // Rotate points back to normal
    x_next_new = (curr_x_ref*cos(ref_yaw) - curr_y_ref*sin(ref_yaw));
    y_next_new = (curr_x_ref*sin(ref_yaw) + curr_y_ref*cos(ref_yaw));

    //Shift back
    x_next_new += ref_x;
    y_next_new += ref_y;

    next_x_vals.push_back(x_next_new);
    next_y_vals.push_back(y_next_new);
  }
}


