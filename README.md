# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)
[image1]: ./img/behavior.png
[image2]: ./img/driving.png
[image3]: ./img/jmt.png
[image4]: ./img/jmt_conditions.png
[image5]: ./img/jmt_conditions_bis.png
[image6]: ./img/jmt_solver.png
[image7]: ./img/overview.png
[image8]: ./img/predictions.png
[image9]: ./img/track.png
[image9]: ./img/trajectories.png

## Project description

<p align="center">
     <img src="./img/driving.png" alt="pipeline" width="50%" height="50%">
     <br>driving.png
</p>

### Overview  

<p align="center">
     <img src="./img/overview.png" alt="pipeline" width="50%" height="50%">
     <br>overview.png
</p>

```cpp
     // --- 6 car predictions x 50 points x 2 coord (x,y): 6 objects predicted over 1 second horizon ---
     map<int, vector<vector<double>>> predictions = generate_predictions(sensor_fusion, car_s, car_d, horizon);

     // --- long time horizon (close to look onwards at 1 sec when possible) analysis for behavior planner ---
     vector<double> frenet_far;
     if (prev_size > 0) // prev_size typically close to 1 sec
     {
          frenet_far = map.getFrenet(previous_path_x[prev_size-1], previous_path_y[prev_size-1], deg2rad(car_yaw));
          car_s = frenet_far[0];
          car_d = frenet_far[1];
     }
     int car_lane = get_lane(car_d);

     vector<vector<double>> targets = behavior_planner_find_targets(sensor_fusion, prev_size, 
                                                                    car_lane, car_s, car_d, car_vel );

     // -- short time horizon for trajectory (re)generation ---
     prev_size = min(prev_size, param_truncated_prev_size);
     vector<double> frenet_close;
     if (prev_size > 0) // prev_size typically close to 100 msec
     {
          frenet_close = map.getFrenet(previous_path_x[prev_size-1], previous_path_y[prev_size-1], deg2rad(car_yaw));
          car_s = frenet_close[0];
          car_d = frenet_close[1];
     }
     car_lane = get_lane(car_d);

     vector<double> costs;
     vector<vector<vector<double>>> trajectories;
     vector<vector<vector<double>>> prev_paths_s;
     vector<vector<vector<double>>> prev_paths_d;

     int target_lane;
     for (int i = 0; i < targets.size(); i++)
     {
          target_lane = targets[i][0];
          double target_vel = targets[i][1];
          double target_time = 2.0; // TODO should be behavior_planner job

          vector<vector<double>> trajectory; // vector of (traj_x, traj_y)

          struct trajectory_jmt traj_jmt;
          // generate JMT trajectory in s and d: converted then to (x,y) for trajectory output
          traj_jmt = generate_trajectory_jmt(target_lane, target_vel, target_time, map, car_x, car_y, car_yaw, 
                      car_s, car_d, previous_path_x, previous_path_y, prev_size, prev_path_s, prev_path_d);
                      
          trajectory = traj_jmt.trajectory;
          prev_paths_s.push_back(traj_jmt.path_s);
          prev_paths_d.push_back(traj_jmt.path_d);

          double cost = cost_function(trajectory, target_lane, target_vel, predictions, sensor_fusion, car_lane);
          costs.push_back(cost);
          trajectories.push_back(trajectory);
     }

     // --- retrieve the lowest cost trajectory ---
     double min_cost = 1e10;
     int min_cost_index = 0;
     for (int i = 0; i < costs.size(); i++)
     {
          if (costs[i] < min_cost)
          {
               min_cost = costs[i];
               min_cost_index = i;
          }
     }
     target_lane = targets[min_cost_index][0];
     target_vel = targets[min_cost_index][1];
     prev_path_s = prev_paths_s[min_cost_index];
     prev_path_d = prev_paths_d[min_cost_index];

```

### Coordinate transforms

cf map.cpp

<p align="center">
     <img src="./img/track.png" alt="pipeline" width="50%" height="50%">
     <br>track.png
</p>

```cpp
vector<double> Map::getXYspline(double s, double d)
{
     s = fmod(s, max_s);
     double x = spline_x(s) + d * spline_dx(s);
     double y = spline_y(s) + d * spline_dy(s);

     return {x,y};
}
```

### Predictions

cf prediction.cpp  

<p align="center">
     <img src="./img/predictions.png" alt="pipeline" width="50%" height="50%">
     <br>predictions.png
</p>

### Behavior planner

cf behavior.cpp  

<p align="center">
     <img src="./img/behavior.png" alt="pipeline" width="50%" height="50%">
     <br>behavior.png
</p>


### Trajectories generation

cf trajectory.cpp  


<p align="center">
     <img src="./img/jmt.png" alt="pipeline" width="50%" height="50%">
     <br>jmt.png
</p>


<p align="center">
     <img src="./img/jmt_conditions.png" alt="pipeline" width="50%" height="50%">
     <br>jmt_conditions.png
</p>

<p align="center">
     <img src="./img/jmt_conditions_bis.png" alt="pipeline" width="50%" height="50%">
     <br>jmt_conditions_bis.png
</p>

```cpp
  double T = target_time; // 2 seconds if car_d center of line

  // si si_dot si_ddot: to be retieved
  double si = prev_path_s[last_point][0];
  double si_dot = prev_path_s[last_point][1];
  double si_ddot = prev_path_s[last_point][2];

  double di = prev_path_d[last_point][0];
  double di_dot = prev_path_d[last_point][1];
  double di_ddot = prev_path_d[last_point][2];

  double sf, sf_dot, sf_ddot;
  double df, df_dot, df_ddot;

  if (target_vel <= 10) // mph
  {
    // special handling at low speed: cf werling paper
    df = di;
    df_dot = 0;
    df_ddot = 0;

    sf_ddot = 0;
    sf_dot = mph_to_ms(target_vel);
    sf = si + 2 * sf_dot * T;
  }
  else
  {
    df = get_dcenter(target_lane);
    df_dot = 0;
    df_ddot = 0;

    sf_ddot = 0;
    sf_dot = mph_to_ms(target_vel);
    sf = si + sf_dot * T;
  }

  vector<double> start_s = { si, si_dot, si_ddot};
  vector<double> end_s = { sf, sf_dot, 0};

  vector<double> start_d = { di, di_dot, di_ddot };
  vector<double> end_d = { df, df_dot, df_ddot};
```


<p align="center">
     <img src="./img/jmt_solver.png" alt="pipeline" width="50%" height="50%">
     <br>jmt_conditions.png
</p>

```cpp
vector<double> JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS
    start - the vehicles start location given as a length three array
            corresponding to initial values of [s, s_dot, s_double_dot]
    end   - the desired end state for vehicle. Like "start" this is a
            length three array.
    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    */

    MatrixXd A(3,3);
    VectorXd b(3);
    VectorXd x(3);

    A <<   pow(T,3),    pow(T,4),    pow(T,5),
         3*pow(T,2),  4*pow(T,3),  5*pow(T,4),
                6*T, 12*pow(T,2), 20*pow(T,3);

    b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T*T), 
         end[1] - (start[1] + start[2]*T), 
         end[2] - start[2];

    x = A.inverse() * b;

    return {start[0], start[1], start[2]/2, x[0], x[1], x[2]};
}
```

### Trajectories cost ranking

cf cost.cp  

<p align="center">
     <img src="./img/trajectories.png" alt="pipeline" width="50%" height="50%">
     <br>trajectories.png
</p>

```cpp
double cost_function(vector<vector<double>> &trajectory, int target_lane, double target_vel, std::map<int, vector<vector<double>>> &predictions, vector<vector<double>> &sensor_fusion, int car_lane)
{
  double cost = 0; // lower cost preferred

  double cost_feasibility = 0; // vs collisions, vs vehicle capabilities
  double cost_safety = 0; // vs buffer distance, vs visibility
  double cost_legality = 0; // vs speed limits
  double cost_comfort = 0; // vs jerk
  double cost_efficiency = 0; // vs desired lane and time to goal

  double weight_feasibility = 100000; // vs collisions, vs vehicle capabilities
  double weight_safety      = 10000; // vs buffer distance, vs visibility or curvature
  double weight_legality    = 1000; // vs speed limits
  double weight_comfort     = 100; // vs jerk
  double weight_efficiency  = 10; // vs target lane, target speed and time to goal

  // 1) FEASIBILITY cost
  if (check_collision(trajectory, predictions)) cost_feasibility += 10;
  if (check_max_capabilities(trajectory)) cost_feasibility += 1;
  cost = cost + weight_feasibility * cost_feasibility;

  // 2) SAFETY cost
  double dmin = get_predicted_dmin(trajectory, predictions);
  if (dmin < param_dist_safety) cost_safety = param_dist_safety - dmin;
  cost = cost + weight_safety * cost_safety;
   
   ...

  return cost;
}
```

### Configurable parameters

cf params.h and params.cpp

```cpp
// Waypoint map to read from
std::string map_file_ = "../data/highway_map.csv";
// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;
// center point of the track
const double param_center_x = 1000;
const double param_center_y = 2000;

const int param_nb_points = 50; // in the trajectory sent to simulator
const double param_dt = 0.02; // 1 point every 0.02 s
const double param_lane_width = 4.0; // meters
const double param_max_speed_mph = 49;
const double param_max_speed = 22; // m.s-1
const double param_max_accel = 10; // m.s-2
const double param_max_jerk  = 10; // m.s-3 average jerk over 1 second
const double param_fov = 70.0; // Field Of View
const double param_max_speed_inc = param_max_accel * param_dt; // m.s-1 per 0.02 sec
const double param_max_speed_inc_mph = ms_to_mph(param_max_speed_inc);
const double param_dist_slow_down = 30; // when a car is 30 m ahead of us => adjust speed if needed
const double param_dist_safety = 3.5; // meters
const double param_dist_collision = 2.75; // meters
// reduce latency reaction, but account for simulator latency ...
// assume 100 ms max simulator latency
const int param_truncated_prev_size = 5;

const bool param_trajectory_jmt = true;

```

### Logs

************** closest object at 11.5728 meters *************  
closestWaypoint=358  
corrected closestWaypoint=359  
error=0.806243 trt_time=37 us (max_error=1.13948 avg_error=0.764559)  
prev_size=47 car_x=1134.5 car_y=1179.96 car_s=358.026 car_d=9.68375 car_speed=46.3776 ref_vel=49  
car_frenet_s=358.093 car_frenet_d=9.79866  
  
lane 0: front -1 at 1e+10 s_meters ; back 6 at 22.8066 s_meters  
lane 1: front 9 at 18.2398 s_meters ; back 1 at 11.2983 s_meters  
lane 2: front -1 at 1e+10 s_meters ; back 7 at 66.2703 s_meters  

=====> dmin = 11.9632  
car_lane=2 target_lane=2 target_vel=49 cost=0  
=====> dmin = 11.9632  
car_lane=2 target_lane=2 target_vel=48.552 cost=4.48  
=====> dmin = 11.9632  
car_lane=2 target_lane=1 target_vel=49 cost=1.2929  
=====> dmin = 11.9632  
car_lane=2 target_lane=1 target_vel=48.552 cost=5.7729  
  
======> CHANGE LANE: lowest cost for target xxx = (target_lane=xxx target_vel=xxx car_lane=xxx cost=xxx)  


### Conclusion and next steps

### References:  
Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame  
https://pdfs.semanticscholar.org/0e4c/282471fda509e8ec3edd555e32759fedf4d7.pdf   
  
Sampling Based Motion Planning for Heavy Duty Autonomous Vehicles  
http://liu.diva-portal.org/smash/get/diva2:1049189/FULLTEXT01.pdf  
  
Towards Fully Autonomous Driving: Systems and Algorithms  
https://www.cs.cmu.edu/~zkolter/pubs/levinson-iv2011.pdf  
  
Vehicle Trajectory Prediction based on Motion Model and Maneuver Recognition  
https://hal.inria.fr/hal-00881100/PDF/IROS13_PIN_161867_.pdf  
  
Jerk Minimization Trajectory  
http://www.shadmehrlab.org/book/minimum_jerk/minimumjerk.htm  

   
## Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

