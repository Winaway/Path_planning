#include "cost.h"

using namespace std;

using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::VectorXd;
using Eigen::Vector2d;

//dmin: distance between the ego_vehicle and near vehicles
double Cost::get_predicted_dmin(TrajectoryXY const &trajectory, std::map<int, vector<Coord> > &predictions)
{
  double dmin = INF;

  std::map<int, vector<Coord> >::iterator it = predictions.begin();
  while(it != predictions.end())
  {
    int fusion_index = it->first;
    //cout << "fusion_index=" << fusion_index << endl;
    vector<Coord> prediction = it->second;

    assert(prediction.size() == trajectory.x_vals.size());
    assert(prediction.size() == trajectory.y_vals.size());

    for (size_t i = 0; i < prediction.size(); i++) { // up to 50 (x,y) coordinates
      double obj_x = prediction[i].x;
      double obj_y = prediction[i].y;
      double ego_x = trajectory.x_vals[i];
      double ego_y = trajectory.y_vals[i];

      double dist = distance(ego_x, ego_y, obj_x, obj_y);
      if (dist < dmin) {
        dmin = dist;
      }
    }
    it++;
  }

  cout << "=====> dmin = " << dmin << endl;
  return dmin;
}


Cost::Cost(TrajectoryXY const &trajectory, Target target, Predictions &predict, int car_lane, int lane_ago)
{
  cost_ = 0; // lower cost preferred

  double cost_feasibility = 0; // vs collisions, vs vehicle capabilities
  double cost_safety = 0; // vs buffer distance, vs visibility
  double cost_legality = 0; // vs speed limits
  double cost_comfort = 0; // vs jerk
  double cost_efficiency = 0; // vs desired lane and time to goal
  //double cost_goal_lane = 0;
  double cost_lane_change = 0;

  std::map<int, vector<Coord> > predictions = predict.get_predictions();

  // 2) SAFETY cost
  // double dmin = get_predicted_dmin(trajectory, predictions);
  // assert(dmin >= 0);
  // if (dmin < PARAM_DIST_SAFETY) {
  //   cost_safety = PARAM_DIST_SAFETY - dmin;
  // } else {
  //   cost_safety = 0;
  // }
  // cost_ = cost_ + PARAM_COST_SAFETY * cost_safety;

  // 3) LEGALITY cost
  cost_ = cost_ + PARAM_COST_LEGALITY * cost_legality;

  // 4) COMFORT cost
  cost_ = cost_ + PARAM_COST_COMFORT * cost_comfort;

  // 5) EFFICIENCY cost
  cost_efficiency = floor((PARAM_FOV - predict.get_lane_free_space(target.lane))/5.0);
  cout<<"cost_efficiency ="<< cost_efficiency << endl;
  cost_ = cost_ + PARAM_COST_EFFICIENCY * cost_efficiency;

  // if(target.lane ==2 || target.lane == 0){
  //   cost_goal_lane = 3;
  // }
  // cost_ = cost_ + PARAM_COST_GOAL * cost_goal_lane;

  //LANE CHANGE cost
  //make sure that the ego_car do not change lane frequently.
  if((target.lane - lane_ago) > 1){
    cost_lane_change = 20;
  }
  cost_ = cost_ + cost_lane_change;
  // cout << "car_lane=" << car_lane << " target.lane=" << target.lane << " target_lvel=" << predict.get_lane_speed(target.lane) << " cost=" << cost_ << endl;
}

Cost::~Cost() {}

double Cost::get_cost()
{
  return cost_;
}
