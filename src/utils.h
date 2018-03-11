#ifndef UTILS_H
#define UTILS_H

#define PI 3.1416
const double MPH_TO_MS =  0.44; //conversion factor

const double max_velocity = 49.5 * MPH_TO_MS; // miles per hour
const double horizon = 50;
const double points_per_second = 50; // The car visits one point every .02 seconds
const double lane_width = 4.0; //in meters

const double min_gap = 50.0; // Minimum distance between two vehicles (gap)
const double sensor_range = 100.0; // Range in which other vehicles are detected [m]
const double safety_distance = 30.0; // There's risk of collision if other cars are closer [meters]
const double safety_distance_behind = 5.0;
const double brake_distance = 30.0; // Break if there is any car closer

// For converting back and forth between radians and degrees.
constexpr double pi() { return PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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


class Vehicle {
 private:
 public:
  double id;
  double x;
  double y;
  double s;
  double d;
  double vx;
  double vy;
  double yaw;
  double lane;
  double speed;

  Vehicle(double x=0, double y=0, double s=0, double d=0) :
    x(x), y(y), s(s), d(d), id(-1) {

    if (d < lane_width) { // left lane
      lane = 0.0;
    }

    else if (d>lane_width && d<2*lane_width) { // center line
      lane = 1.0;
    }

    else if (d>2*lane_width) { //right lane
      lane = 2.0;
    }
  }

  void setVelocity(double vx, double vy);
  void setId(double id);
  double getSpeed();
  double getLane();
};

void Vehicle::setVelocity(double vx, double vy) {
    this->vx = vx;
    this->vy = vy;
    speed = sqrt(vx*vx + vy*vy);
  }
void Vehicle::setId(double id) {
  this->id = id;
}
double Vehicle::getSpeed() {
  return speed;
}

class VehicleDetector {
 private:
  Vehicle ego;
  double lane;

  Vehicle car_infront;

 public:
  VehicleDetector::VehicleDetector(Vehicle ego, double lane, vector<vector<double>> sensor_fusion);

  bool static infront_tooClose;
  double static closest_car_distance;

  // Vehicles around ego car
  vector<Vehicle> cars_left_ahead;
  vector<Vehicle> cars_center_ahead;
  vector<Vehicle> cars_right_ahead;

  vector<Vehicle> cars_left_behind;
  vector<Vehicle> cars_center_behind;
  vector<Vehicle> cars_right_behind;

  Vehicle getCarInfront() {
    if (infront_tooClose) {
      return car_infront;
    }
    else {
      return 0;
    }
  }

};

VehicleDetector::infront_tooClose = false;
VehicleDetector::closest_car_distance = 2000000000;

VehicleDetector::VehicleDetector(Vehicle ego, double lane, vector<vector<double>> sensor_fusion) {

  this->ego = ego;
  this->lane = lane;

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
    if (other.lane == 0.0) {
      if (is_ahead && is_detectable) {
        cars_left_ahead.push_back(other);
      }
      else if (!is_ahead && is_detectable) {
        cars_left_behind.push_back(other);
      }
    }

    else if (other.lane == 1.0) {
      if (is_ahead && is_detectable) {
        cars_center_ahead.push_back(other);
      }
      else if (!is_ahead && is_detectable) {
        cars_center_behind.push_back(other);
      }
    }

    else if (other.lane == 2.0) {
      if (is_ahead && is_detectable) {
        cars_right_ahead.push_back(other);
      }
      else if (!is_ahead && is_detectable) {
        cars_right_behind.push_back(other);
      }
    }

    // Check if the other car is in our lane
    // If it is too close, we need to change lane!
    if (other.lane == lane) {
      double distance = abs(other.s-ego.s);
      infront_tooClose = (is_ahead) && (distance < min_gap);

      if (infront_tooClose) {
        car_infront = other;

        // Update the distance with the car in front
        if (distance < closest_car_distance){
          closest_car_distance = distance;
        }
      }
    }
  }
}



#endif /* UTILS_H */
