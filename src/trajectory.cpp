#include "trajectory.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;




Trajectory::Trajectory(std::vector<Target> targets, Map &map, CarData &car, PreviousPath &previous_path, Predictions &predictions)
{
  //int lane_1s_ago = car.passed_path.front();
  for (size_t i = 0; i < targets.size(); i++) {
    TrajectoryXY trajectory;
    trajectory = generate_trajectory(targets[i], map, car, previous_path);

    Cost cost = Cost(trajectory, targets[i], predictions, car.lane,car.passed_path.front());
    costs_.push_back(cost);
    trajectories_.push_back(trajectory);
  }

  // --- retrieve the lowest cost trajectory ---
  min_cost_ = INF;
  min_cost_index_ = 0;
  for (size_t i = 0; i < costs_.size(); i++) {
    if (costs_[i].get_cost() < min_cost_) {
      min_cost_ = costs_[i].get_cost();
      min_cost_index_ = i;
    }
  }
}


TrajectoryXY Trajectory::generate_trajectory(Target target, Map &map, CarData const &car, PreviousPath const &previous_path)
{
  TrajectoryXY previous_path_xy = previous_path.xy;
  int prev_size = previous_path.num_xy_reused;

  vector<double> previous_path_x = previous_path_xy.x_vals;
  vector<double> previous_path_y = previous_path_xy.y_vals;

  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car.x;
  double ref_y = car.y;
  double ref_yaw = deg2rad(car.yaw);

  if (prev_size < 2) {
    double prev_car_x = car.x - cos(car.yaw);
    double prev_car_y = car.y - sin(car.yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car.x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car.y);
  } else {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  vector<double> next_wp0 = map.getXY(car.s+30, get_dcenter(target.lane));
  vector<double> next_wp1 = map.getXY(car.s+60, get_dcenter(target.lane));
  vector<double> next_wp2 = map.getXY(car.s+90, get_dcenter(target.lane));

  // vector<double> next_wp0 = map.getXYspline(car.s+30, get_dcenter(target.lane));
  // vector<double> next_wp1 = map.getXYspline(car.s+60, get_dcenter(target.lane));
  // vector<double> next_wp2 = map.getXYspline(car.s+90, get_dcenter(target.lane));


  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);


  for (int i = 0; i < ptsx.size(); i++) {
    // shift car reference angle to 0 degrees
    // transformation to local car's coordinates (cf MPC)
    // last point of previous path at origin and its angle at zero degree

    // shift and rotation
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsy[i]-ref_y;

    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0 - ref_yaw));
  }


  tk::spline spl;
  spl.set_points(ptsx, ptsy);

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  for (int i = 0; i < prev_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  // fill up the rest of our path planner after filing it with previous points
  // here we will always output 50 points
  for (int i = 1; i <= PARAM_NB_POINTS - prev_size; i++) {
    double N = (target_dist / (PARAM_DT * mph_to_ms(target.velocity))); // divide by 2.24: mph -> m/s
    double x_point = x_add_on + target_x/N;
    double y_point = spl(x_point);

    x_add_on = x_point;

    double x_ref = x_point; // x_ref IS NOT ref_x !!!
    double y_ref = y_point;

    // rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  //return { next_x_vals, next_y_vals };
  return TrajectoryXY(next_x_vals, next_y_vals);
}
