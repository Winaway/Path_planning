#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <math.h>
#include <iostream>
#include <vector>
#include <cassert>

#include "params.h"
#include "utility.h"
#include "predictions.h"

struct Target {
  double lane;
  double velocity;
  // double accel; // XXX for emergency trajectories
  Target(double l=0, double v=0) : lane(l), velocity(v) {}
};


class Behavior {
public:
  Behavior(std::vector<std::vector<double>> const &sensor_fusion, CarData car, Predictions const &predictions);
  virtual ~Behavior();
  std::vector<Target> get_targets();

private:
  std::vector<Target> targets_;
};


#endif // BEHAVIOR_H
