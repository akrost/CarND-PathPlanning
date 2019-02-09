# CarND-Path-Planning-Project

[//]: # (Image References)
[img_hold_lane]: ./img/01_example.png "Car holding lane"
[img_change_lane]: ./img/02_example.png "Car changing lanes"

![Car holding lane][img_hold_lane]

### Simulator.
The Simulator can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


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

## Components

### DriveController

The DriveController (DC) is the main component of the path planner. It creates a trajectory based on the current environment state.

When the class is initialized, the `Map` class and the `Car` class are stored in the DC:

```C++
Map highway_map = Map();
Car car = Car();
DriveController drive_controller = DriveController(&car, &highway_map);
```

The sensor fusion data (positions of the cars surrounding the ego car) has to be updated every cycle:

```C++
drive_controller.update_environment(sensor_fusion);
```

The same applies for the current car status:

```C++
drive_controller.car->setX(j[1]["x"]);
drive_controller.car->setY(j[1]["y"]);
drive_controller.car->setS(j[1]["s"]);
drive_controller.car->setD(j[1]["d"]);
drive_controller.car->setYaw(j[1]["yaw"]);
drive_controller.car->setSpeed(j[1]["speed"]);
```

After the current environment and the car status are updated, the DC can generate a trajectory. To do that, the `plan_path()` function is called.

```C++
vector<vector<double>> next_vals = drive_controller.plan_path(previous_path_x.size());
```

It handles acceleration, checks if lanes are safe and performs lane changes.

Then, the trajectory is generated in the `vector<vector<double>> DriveController::create_trajectory(int prev_size)` function. It generates 5 anchor points (in car centered coordinate system) and then creates a spline using the anchor points. Using the spline, a set number of trajectory points can be generated. Finally, the points have to be transformed back into a global coordinate system. The spline is used to smooth the trajectory in order to stay in the jerk boundaries.

### Car

The `Car` class is a data class that stores the current position, speed and lane of the ego car.

### Map

The `Map` class is a data class that stores the map attributes (x, y, s, dx, dy).

![Car changing lanes][img_change_lane]
