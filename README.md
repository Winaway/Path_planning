# Path Planning Project

## Introduction
The goal of this project is to navigate a car around a simulated highway scenario, including traffic and given waypoint, telemetry, and sensor fusion data. The car must not violate a set of motion constraints, namely maximum velocity, maximum acceleration, and maximum jerk, while also avoiding collisions with other vehicles, keeping to within a highway lane (aside from short periods of time while changing lanes), and changing lanes when doing so is necessary to maintain a speed near the posted speed limit.

The implementation is divided into six parts:
1. map.cpp:  Read map from file, set the waypoint.
2. predictions.cpp: Generate predictions from sensor fusion data.
3. behavior.cpp: Generate serval targets.
4. cost.cpp: Calculate the cost of candidate trajectory.
5. trajectory.cpp: Generate  trajectories & Choose the best trajectory.
6. main.cpp.

## Implementation

### 1. Read map from file, set the waypoint.
Load in the waypoints from  the `highway_map.csv` file. Include other four functions:
 `ClosestWaypoint` : Find the closest way point on the map;
 `NextWaypoint` : Get the next way point;
 `getFrenet` :  Transform from Cartesian x,y coordinates to Frenet s,d coordinates
 `getXY` : Transform from Frenet s,d coordinates to Cartesian x,y.


### 2. Generate predictions from sensor fusion data & Set lane information.
1. At first, find the closest vehicles(at most six) near ego_car; 
2. According to the sensor_fusion data, we can set the safety distance of each nearby vehicle;
3. Combined with the safety distance and nearby vehicles state, set the lane_speed_ and lane_free_space_ of each lane.

### 3. Generate several targets.
we aimed to generate next targets (target_speed, target_lane) for ego_car.
At first, we get the target_speed based on the speed of front vehicle in the current lane and safety_distance. Secondly, Traverse all possible values of target_lane. eg. current_lane = 0, possible target lanes are lane_0 and lane_1. 

### 4. Generate the trajectory based on targets.
In the `trajectory.cpp` module, we have a function to generate trajectory according to a target. In this function we used `spline.h` to generate smooth curves, which help to  minimize the jerk of the path. Additionally, we use the remaining previous path point to generate new path, so we can make sure the smooth transition between two paths. 

### Calculate the cost of candidate trajectory.
we have a cost function to calculate the cost of each candidate trajectory. the total cost is made up of several parts, including safety_cost, efficiency_cost  and lane_change_cost.
safety_cost aims to avoid collision with other cars in lane_change process.
efficiency_cost  make sure that the ego_car can travel as faster as possible safely. 
lane_change_cost aims to decrease the fluency of change lane.

### Choose the best trajectory.
Combined the generated target_trajectories and cost function, we can choose a trajectory with the lowest cost as the best trajectory.

## Conclusion
The final path planner works well, but not perfectly. The car is able to drive at least 4.32 miles without incident, but the planner can avoid maximum jerk at special situation. Improving the planner proved to be very difficult , because there  are  always  suddenly  acceleration or deceleration of  other vehicles.

![](Path%20Planning%20Project/Snip20190127_3.png)

::belowing are from Udacity::
# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m_s^2 and jerk that is greater than 10 m_s^3.

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

Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m_s, car's y velocity in m_s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m_s^2, also the jerk should not go over 50 m_s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

- - - -

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

* _ide_profiles_vscode/.vscode
* _ide_profiles_vscode/README.md

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