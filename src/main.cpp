#include <fstream>
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

#define MPH_TO_MS 0.44 //conversion factor

const double max_velocity = 49.8 * MPH_TO_MS; // miles per hour
const double horizon = 50;
const double points_per_second = 50; // The car visits one point every .02 seconds
const double lane_width = 4.0; //in meters
const double security_distance = 30; // There's risk of collision if other cars are closer [meters]

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Euclidean distance between two points (x1, x2), (y1, y2)
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculates the closest waypoint (maps_x[i], maps_y[i]), with respect to our point (x, y)
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	// Difference between our current direction, and the direction we should go next (heading)
	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  // Changes in the direction of our car of more than pi/4 rads are not desired
  if(angle > pi()/4)
  {
    // Get the next waypoint, ??
    closestWaypoint++;
    // Unless it is the last waypoint
    if (closestWaypoint == maps_x.size())
    {
      // Our current
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
	  // The circuit is closed and the cars should drive in a loop
	  // If the next waypoint is the first (0), the the previous wp is the last one!
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

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
  double lane = 1.0; // Double to ease maths

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

          	// Number of waypoints calculated in the last iteration, which the car didn't go through
          	// Would be a number close to 50 (e.g 47)
          	double prev_size = previous_path_x.size();

          	// Detect collisions
          	if (prev_size > 0) {
          	  car_s = end_path_s; // Use last point of the previous path as car_s to ease calculations
          	}

          	bool too_close = false;

          	// Check other cars in the road
          	for (int i=0; i<sensor_fusion.size(); ++i) {
          	  double other_car_d = sensor_fusion[i][6];

          	  // If the other car is on our same lane
          	  if (other_car_d>lane*lane_width && other_car_d<(lane+1)*lane_width) {
          	    // Check how close the other car is
          	    double other_car_s = sensor_fusion[i][5];

          	    double other_car_vx = sensor_fusion[i][3];
          	    double other_car_vy = sensor_fusion[i][4];
          	    double other_car_v = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);

          	    // Predict other car's position into the future
          	    // To be consistent with our car_s (which could also be in the future)
          	    other_car_s += other_car_v*prev_size/points_per_second;

          	    if (other_car_s>car_s && (other_car_s-car_s)<security_distance) {
          	      too_close = true;

          	      // Logic about lane changing
          	      if (lane > 0) {
          	        lane = 0;
          	      }
          	    }

          	  }
          	}

          	// Reduce target velocity if we are too close to other cars
          	if (too_close) {
          	  //lane = 1;
          	  ref_velocity -= .6; // m/s
          	} else if (ref_velocity < max_velocity){
          	  ref_velocity += .6;
          	}



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

          // Second, add three "anchor" points, located at 30, 60 and 90 meters from the car's current position
          // Note that the reference velocity is ~25m/s, which means that the three anchor points are not part of
          // the previous trajectory points (50 points ~ 1s ~ 25m )
          // This also helps to find a smooth trajectory
          vector<double> next_wp0 = getXY(car_s+ 30, (lane*lane_width + lane_width/2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+ 60, (lane*lane_width + lane_width/2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+ 90, (lane*lane_width + lane_width/2), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // The trajectory points must be evenly located, to make the car travel at the desired velocity
          // and they must fit in the trajectory of our previously calculated points.
          // To fit them, we use spline http://kluge.in-chemnitz.de/opensource/spline/

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

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

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

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          	/*The car reaches a point each 0.02 seconds (50 points per second)
          	  If we want a velocity of 50 mph (22 m/s aprox), then the distance between points shoud be:
          	  distance[m/points] = ref_velocity[m/s] * 0.02[points/s]
          	*/
          /*
           ----- FIRST APPROACH -----

          // Calculate the distance between points to reach a desired velocity
          	double step = ref_velocity / points_per_second;

          for(int i = 0; i < horizon; ++i)
          {
            // Define in Frenet where we want to go
            double next_s = car_s + (i+1)*step;
            double next_d = lane*lane_width + lane_width/2; // Add a half more to stay on the middle!

            vector<double> next_xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(next_xy[0]);
            next_y_vals.push_back(next_xy[1]);

          }

           */

          	// END
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
