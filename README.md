# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Project reflection

---

### 1. The Model
_(Student describes their model in detail. This includes the state, actuators and update equations.)_

The kinematic model is the one used in class, with a minor modification.

__The states:__
* _x_ , the position of the car along the x axis in global coordinates
* _y_ , the position of the car along the y axis in global coordinates
* _psi_ , the counter-clockwise angle of the car around the z axis
* _v_ , the current vehicle speed
* _cte_ , the cross-track error from the desired trajectory
* _epsi_ , the orientation error from the desired orientation, the tangent of the trajectory

__The actuator values:__
* _delta_ , the steering wheel angle, between -25° and 25°
* _a_ , the throttle and brake pedals, between 1 (full throttle) and -1 (full brake)

__The update equations:__

![](https://latex.codecogs.com/gif.latex?x_%7Bt&plus;1%7D%20%3D%20x_t%20&plus;%20v_t%20*%20%5Ccos%28%5Cpsi_t%29%20*%20dt)

![](https://latex.codecogs.com/gif.latex?y_%7Bt&plus;1%7D%20%3D%20y_t%20&plus;%20v_t%20*%20%5Csin%28%5Cpsi_t%29%20*%20dt)

![](https://latex.codecogs.com/gif.latex?%5Cpsi_%7Bt&plus;1%7D%20%3D%20%5Cpsi_t%20-%20%5Cfrac%20%7Bv_t%7D%20%7B%20L_f%7D%20*%20%5Cdelta%20*%20dt)

![](https://latex.codecogs.com/gif.latex?v_%7Bt&plus;1%7D%20%3D%20v_t%20&plus;%20a_t%20*%20dt)

![](https://latex.codecogs.com/gif.latex?cte_%7Bt&plus;1%7D%20%3D%20y_t%20-%20f%28x_t%29%20&plus;%20%28v_t%20*%20sin%28e%5Cpsi_t%29%20*%20dt%29)

![](https://latex.codecogs.com/gif.latex?e%5Cpsi_%7Bt&plus;1%7D%20%3D%20%5Cpsi_t%20-%20%5Cpsi%7Bdes%7D_t%20&plus;%20%28%5Cfrac%7Bv_t%7D%20%7B%20L_f%7D%20*%20%5Cdelta_t%20*%20dt%29)

The increment of psi needs to be negative to counter the fact that in the simulator, psi and delta have opposite directions.

__Cost function members:__
* Difference from reference signals (cross-track error, psi error, reference speed)
* Actuator values (steering and throttle)
* Actuator rates (rates of steering and throttle)

All these members had their respective weights to chip in to the sum of cost in a comparable level.

---

### 2. Timestep Length and Elapsed Duration (N & dt)
_(Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.)_

After trying the values shown in the lessons (N = 25, dt = 0.05), it quickly turns out, that picking a too high N and too low dt makes the calculation overly complicated and thus causing performance problems. It increases the horizon of the cost function too, which means that some features of the trajectory are reckoned with that are not yet relevant.

Based on the walkthrough recommendation, I ended up with the values N = 10 and dt = 0.1, giving a prediction horizon of 1 second. Increasing or decreasing these values did not yield better results, so I left them as they are.

---

### 3. Polynomial Fitting and MPC Preprocessing
_(A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.)_

There is one important preprocessing step that can make our life easier in this project. It is to transform the waypoints from map coordinates to car coordinates. It can easily be done on the batch of received waypoints, and then the polynomial fitting becomes a lot simpler. When using car coordinates, the x, y and psi variables turn into zeros and so the higher cross-track error calculation terms and psi error calculation terms can be eliminated.

See [line 99](https://github.com/rdhelli/CarND-MPC-Project/blob/master/src/main.cpp#L110) of `main.cpp`

---

### 4. Model Predictive Control with Latency
_(The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.)_

One of the ways latency can be handled is to make a future-forward simulation based on the current state and initialize the MPC solver with the resulting simulated state. We can apply the kinematic motion model to make an estimation of where the car will be at the time our commands get actuated, thus the MPC can explicitly take the 100 millisecond latency into account.

See [line 110](https://github.com/rdhelli/CarND-MPC-Project/blob/master/src/main.cpp#L110) of `main.cpp`

Another advantage of the above explained preprocessing, is that in car coordinates, the motion model calculation simplifies as well, with the coordinate variables being zero.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
