#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <iostream>
#include <vector>

#include "map.h"
#include "behavior.h"
#include "spline.h"
#include "utility.h"
#include "map.h"
#include "cost.h"
#include "params.h"
#include "predictions.h"

#include "Eigen-3.3/Eigen/Dense"



struct TrajectoryXY {
  std::vector<double> x_vals;
  std::vector<double> y_vals;
  TrajectoryXY (std::vector<double> X={}, std::vector<double> Y={}) : x_vals(X), y_vals(Y) {}
};

struct PreviousPath {
  TrajectoryXY xy;   // < PARAM_NB_POINTS (some already used by simulator)
  int num_xy_reused;  // reused from xy
  PreviousPath (TrajectoryXY XY={}, int N=0) : xy(XY),  num_xy_reused(N) {}
};

class Trajectory {
public:
  Trajectory(std::vector<Target> targets, Map &map, CarData &car, PreviousPath &previous_path, Predictions &predictions);
  ~Trajectory() {};

  double getMinCost() { return min_cost_; };
  double getMinCostIndex() { return min_cost_index_; };
  TrajectoryXY getMinCostTrajectoryXY() { return trajectories_[min_cost_index_]; };

private:
  std::vector<class Cost> costs_;
  std::vector<TrajectoryXY> trajectories_;
  double min_cost_;
  int min_cost_index_;


  TrajectoryXY generate_trajectory (Target target, Map &map, CarData const &car, PreviousPath const &previous_path);
};

#endif // TRAJECTORY_H
