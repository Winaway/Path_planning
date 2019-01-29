#include "cost.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::VectorXd;
using Eigen::Vector2d;


Cost::Cost(TrajectoryXY const &trajectory, Target target, Predictions &predict, int car_lane, int lane_ago)
{
  cost_ = 0; // lower cost preferred

  double cost_safety = 0;
  double cost_efficiency = 0;
  double cost_efficiency_2 = 0;
  double cost_lane_change = 0;

  std::map<int, vector<Coord> > predictions = predict.get_predictions();

  // 1) SAFETY cost
  if (predict.get_lane_free_space(target.lane)==0) {
    cost_safety = 1;
  } else {
    cost_safety = 0;
  }
  cost_ = cost_ + PARAM_COST_SAFETY * cost_safety;


  // 2) EFFICIENCY cost
  cost_efficiency = floor((PARAM_FOV - predict.get_lane_free_space(target.lane))/5.0);
  cout<<"cost_efficiency ="<< cost_efficiency << endl;
  cost_ = cost_ + PARAM_COST_EFFICIENCY * cost_efficiency;

  cost_efficiency_2 = PARAM_MAX_SPEED - predict.get_lane_speed(target.lane);
  cost_ = cost_ + PARAM_COST_EFFICIENCY_2 * cost_efficiency_2;


  // 3) LANE CHANGE cost
  // Make sure that the ego_car do not change lane frequently.
  if((target.lane - lane_ago) > 1){
    cost_lane_change = 20;
  }
  cost_ = cost_ + cost_lane_change;
}

Cost::~Cost() {}

double Cost::get_cost()
{
  return cost_;
}
