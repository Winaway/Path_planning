#ifndef MAP_H
#define MAP_H

#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>

#include "spline.h"

class Map {

public:
  /**
  * Constructor
  */
  Map() {};

  /**
  * Destructor
  */
  virtual ~Map();

  std::vector<double> getFrenet(double x, double y, double theta);
  std::vector<double> getXY(double s, double d);

  void read(std::string map_file);

private:
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
  int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
};

#endif // MAP_H
