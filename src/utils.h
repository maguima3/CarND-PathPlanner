#ifndef UTILS_H
#define UTILS_H

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
using namespace std;

#define PI 3.1416
const double MPH_TO_MS =  0.44; //conversion factor

const double max_velocity = 49.5 * MPH_TO_MS; // miles per hour
const double horizon = 50;
const double points_per_second = 50; // The car visits one point every .02 seconds
const double lane_width = 4.0; //in meters

const double min_gap = 25.0; // Minimum distance between two vehicles (gap) // 50?
const double sensor_range = 50.0; // Range in which other vehicles are detected [m] // 100?
const double safety_distance = min_gap; // There's risk of collision if other cars are closer [meters] //  30?
const double safety_distance_behind = 6.0;//
const double brake_distance = 30.0; // Break if there is any car closer.
const double danger_distance = 10.0;

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// Euclidean distance between two points (x1, x2), (y1, y2)
double distance(double x1, double y1, double x2, double y2);

// Calculates the closest waypoint (maps_x[i], maps_y[i]), with respect to our point (x, y)
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

#endif /* UTILS_H */
