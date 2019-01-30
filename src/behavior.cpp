#include "behavior.h"

using namespace std;

// generate next targets(target_speed,target_lane) for ego_car.
// get the target_speed based on the speed of front vehicle in the current lane and safety_distance.
// Traverse all possible values of target_lane. eg. current_lane = 0,possible target lanes are lane_0 and lane_1. 
Behavior::Behavior(vector<vector<double>> const &sensor_fusion, CarData car, Predictions const &predictions) {
  Target target;
  double car_speed_target = car.speed_target;

  double safety_distance = predictions.get_safety_distance();

  bool too_close = false;

  double ref_vel_ms = mph_to_ms(car_speed_target);
  double closest_speed_ms = PARAM_MAX_SPEED;
  double closest_dist = INF;

  // find ref_v to use based on car in front of us
  //找到当前车道里目标车辆前面最近的车辆
  for (size_t i = 0; i < sensor_fusion.size(); i++) {
    // car is in my lane
    float d = sensor_fusion[i][6];
    if (d > get_dleft(car.lane) && d < get_dright(car.lane)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5];


      //cout << "obj_idx=" << i << " REF_VEL_MS=" << ref_vel_ms << " CHECK_SPEED=" << check_speed << endl;

      if ((check_car_s > car.s) && ((check_car_s - car.s) < safety_distance)) {
        // do some logic here: lower reference velocity so we dont crash into the car infront of us
        //ref_vel = 29.5; //mph
        too_close = true;
        double dist_to_check_car_s = check_car_s - car.s;
        if (dist_to_check_car_s < closest_dist) {
          closest_dist = dist_to_check_car_s;
          closest_speed_ms = check_speed;
        }
      }
    }
  }

  if (too_close) {
    if (ref_vel_ms > closest_speed_ms) { // in m.s-1 !
      car_speed_target -= 0.6*PARAM_MAX_SPEED_INC_MPH; // in mph ! 当离前面的车辆非常近的时候，已更高的减速度刹车
      if (closest_dist <= 20 && car_speed_target > closest_speed_ms) {
        car_speed_target -= 0.9*PARAM_MAX_SPEED_INC_MPH;
      }
    }
    car_speed_target = max(car_speed_target, 0.0); // no backwards driving ... just in case 确保不会倒车
  } else if (car_speed_target < PARAM_MAX_SPEED_MPH) {
    //ref_vel += 2 * .224;
    car_speed_target += 0.9*PARAM_MAX_SPEED_INC_MPH;
    car_speed_target = min(car_speed_target, PARAM_MAX_SPEED_MPH);//确保速度不会超过限速
  }

  // our nominal target .. same lane
  target.lane = car.lane;
  target.velocity = car_speed_target;

  targets_.push_back(target);

  vector<int> backup_lanes;
  switch (car.lane)
  {
    case 2:
      backup_lanes.push_back(1);
      break;
    case 1:
      backup_lanes.push_back(2);
      backup_lanes.push_back(0);
      break;
    case 0:
      backup_lanes.push_back(1);

      break;
    default:
      assert(1 == 0); // something went wrong
      break;
  }

  // 2) target velocity on backup lanes
  target.velocity = car_speed_target;
  for (size_t i = 0; i < backup_lanes.size(); i++) {
    target.lane = backup_lanes[i];
    targets_.push_back(target);
  }
}

Behavior::~Behavior() {}

vector<Target> Behavior::get_targets() {
  return targets_;
}
