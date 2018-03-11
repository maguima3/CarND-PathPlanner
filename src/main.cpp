#include <fstream>
#include <stdio.h>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "spline.h"
#include "utils.h"
#include "trajectory_generator.cpp"
#include "path_planner.cpp"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "/Users/mariaguinea/Workspace/CarND/Term3/Projects/1_PathPlanning/data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  double ref_velocity = 1.0; // Start at 1 m/s
  double lane = 1.0; // Double to ease math

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &ref_velocity, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	/*
          	 * 2d vector of cars and then that car's:
          	 *  car's unique ID,
          	 *  car's x position in map coordinates,
          	 *  car's y position in map coordinates,
          	 *  car's x velocity in m/s,
          	 *  car's y velocity in m/s,
          	 *  car's s position in frenet coordinates,
          	 *  car's d position in frenet coordinates.
          	 */
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	// Define our car
          	Vehicle ego = Vehicle(car_x, car_y, car_s, car_d);
          	ego.setId(0);
          	ego.speed = car_speed;
          	ego.yaw = car_yaw;

          	// Number of waypoints calculated in the last iteration, which the car didn't go through
          	// Would be a number close to 50 (e.g 47)
          	double prev_size = previous_path_x.size();

          	// Detect collisions
          	if (prev_size > 0) {
          	  car_s = end_path_s; // Use last point of the previous path as car_s to ease calculations
          	}

          	VehicleDetector detector = VehicleDetector(ego, lane, sensor_fusion);
          // Vehicles detected around our car
          vector<Vehicle> cars_left_ahead = detector.cars_left_ahead;
          vector<Vehicle> cars_center_ahead = detector.cars_center_ahead;
          vector<Vehicle> cars_right_ahead = detector.cars_right_ahead;
          vector<Vehicle> cars_left_behind = detector.cars_left_behind;
          vector<Vehicle> cars_center_behind = detector.cars_center_behind;
          vector<Vehicle> cars_right_behind = detector.cars_right_behind;

          bool change_lane = detector.infront_tooClose;
          Vehicle car_infront = detector.getCarInfront();

          PathPlanner planner = PathPlanner(ego, detector, car_s, prev_size);
          lane = planner.nextLane();

          /*
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
          	*/

          // Reduce target velocity if we are too close to other cars
          if (detector.closest_car_distance < brake_distance) {

            if (ref_velocity > car_infront.getSpeed()) {
              ref_velocity -= .2; // m/s

            } else {
              ref_velocity += .2; // m/s
            }

          } else if (ref_velocity < max_velocity){
            ref_velocity += .6;
          }


          // Trajectory generation

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          TrajectoryGenerator trajectoryGenerator = TrajectoryGenerator(car_x, car_y, car_yaw, car_s,
                                                                        ref_velocity, lane,
                                                                        map_waypoints_x, map_waypoints_y, map_waypoints_s,
                                                                        previous_path_x, previous_path_y);

          trajectoryGenerator.findTrajectory(next_x_vals, next_y_vals);

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
