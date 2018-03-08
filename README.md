# Model Predictive Controller

[//]: # (Image References)
[image1]: ./images/sim01.png
[image2]: ./images/mpc.png

## Overview

This project implements a Model Predictive Controller (MPC) using the kinematic model for the vehicle's motion. It works in conjunction with Udacity's SDC Simulator. The simulator feeds the model with the reference trajectory. The controller uses the kinematic equations to determine optimal actuation values (steering and throttle/brake) to keep the car on the desired trajectory. The actuation commands are sent back to the simulator, and the process repeats. The image below shows this in action. The yellow line is the reference trajectory (in the vehicle's coordinate system) and the green line is the path calculated by the model. After tuning the MPC, the car reached a stop speed of 79mph.

![simulation][image1]
  
  
## MPC Methodology

The image below shows the implemented model, composed of: time discretization, kinematic equations including errors, and actuator bounds.

![mpc][image2]

### Vehicle State Representation

The vehicle's state is modeled using 6 variables: x, y, psi, v, cte and epsi. The first four variables (x, y, psi, v) represent the vehicle's x-y coordinates, its orientation, and its velocity respectively. The last two variables (cte, epsi) represent the cross track error and orientation error based on the reference trajectory received from the simulator. These state variables are updated using the kinematic equations in the image above. The constraints in the model above are actually actuation variables for steering (delta) and throttle/brake (a) that the MPC determines through cost optimization, within given bounds.

### Model Cost

Collectively the model equations and actuation bounds form the constraints through which the model's cost is optimized (minimized). This is done using the ipopt and CppAD libraries. The model's cost is based on four main considerations: 
* cte and epsi should be minimized to stay as close to the reference trajectory as possible.
* Magnitude of actuator variables should be minimized to avoid erratic motion.
* Rate of change between successive actuations should be minimize for smoother transitions.
* The vehicle should maintain (or be close to) a certain reference velocity, which is set at 80mph.
  
The equations that model the cost function are tuned through multipliers. Certain cost contributors, such as cross track and psi error, are given a higher priority (large multiplier) in order to keep the vehicle on track. This also causes the vehicle to slowdown during sharp turns. This is especially important at high speeds where the steering angle has a pronounced effect on the car's position.

### Time Discretization

The ipopt optimizer works on discrete values and as such time is modelled as discrete time-steps represented by N (number of steps) and dt (elapsed time between time). Multiplying these two values gives the total time the model 'looks ahead'. Using these two hyperparameters, the MPC determines the lowest cost trajectory at each time step, and the actuations required for them. Note: only actuations from the first (current) time-step are sent to the simulator.

These hyperparameters are tuned in order to produce stable vehicle motion. Larger values of N requires the optimizer to consider more time-steps in its calculation, which is a time intensive process. Since state values further in the future are less reliable, a small value is chosen for N. For dt smaller values are desired, since it makes the MPC's behaviour more continuous. However it also shortens the controllers horizon (T = N * dt). In the implemented controller N = 10 time-steps and dt = 0.1s. Other values were tested but they either introduced computational delay (higher N) or a short horizon (smaller dt).

### Trajectory Preprocessing and Polynomial Fitting

The MPC receives the reference trajectory from the simulator in the global coordinate system. Since the vehicle's state is represented in the vehicle's coordinate system, all global reference points are first converted. Afterwards the transformed reference points are fitted to a 3rd degree polynomial to approximate the reference trajectory. This polynomial (represented by its coefficients) allows us to easily calculate model variables for future states.

### Actuator Latency

In an actual Self Driving Car the actuators have a systemic delay, from the time the command is issues to when its actually performed. This is a characteristic of the mechanical components involved in performing these commands. One advantage of MPC is that it allows us to easily account for this latency. 

In the program this is modelled by a 100ms delay between when the MPC calculates the actuations and when they're sent back to the simulator (using thread sleep). In order to account for this latency a delayed state is calculated for the vehicle, using the kinematic equations, that determines where the car will be after 100ms. This delayed state (x', y', psi', v', cte', epsi') is then sent to the MPC to determine desired actuations 100ms into the future. This results in more stable vehicle motion, since the effects of latency can build up in wild oscillations.
  
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
