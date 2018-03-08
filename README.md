# Model Predictive Controller for Udacity's Self Driving Car Simulator

[//]: # (Image References)
[image1]: ./images/sim01.png
[image2]: ./images/mpc.png

## Overview

This project implements a Model Predictive Controller (MPC) using the kinematic model for the vehicle's motion. It works in conjunction with Udacity's SDC Simulator. The simulator feeds the model with the desired trajector. The controller program uses the kinematic equations to determine desired acuations (steering and throttle/brake) to keep the car on the desired tradjector. The actuation commands are sent back to the simulator, and the process repeats. The image below shows this in action. The yellow line is the reference tradjector (in the vehicle's coordinate system) and the green line is the path calculated by the model.

![simulation][image1]

## MPC Methodology

The image below shows the implemented model composed of time discretization, kinematic equations including errors, and actuator constraints.

![mpc][image2]

### Vehicle State Representation

### Time Discretization

### Tradjector Preprocessing and Polynomial Fitting

### Actuator Latency

## Tuning

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


## Build Instructions

1. Clone this repo.
2. Change directory into the project folder.
3. Make a build directory: `mkdir build && cd build`
4. Compile: `cmake .. && make`
5. Run it: `./mpc`.
6. Launch Udacity's SDC Simulator and start `Project 5: MPC Controller`.

