#ifndef COST_H
#define COST_H

#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <cassert>
#include <cmath>

#include "utility.h"
#include "params.h"
#include "predictions.h"
#include "Eigen-3.3/Eigen/Dense"

#include "behavior.h"
#include "trajectory.h"



class Cost {
public:
  Cost(struct TrajectoryXY const &trajectory, Target target, Predictions &predictions, int car_lane,int lane_ago);
  virtual ~Cost();

  double get_cost();

private:
  double cost_;
};

#endif // COST_H
