# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Intro

This repo contains my work on the final project for Term 2 the Udacity Self-driving Car Nanodegree. In this project, students were tasked with creating a model predictive controller and to use it to navigate around Udacity's lake track driving simulator.

## The Model

### Vehicle State

In order to keep things simple, the model for this project is a kinematic model which considers only six vehicle state variables:

* X - the vehicle's projected position in relation to its current position and orientation on the x axis
* Y - the vehicle's projected position in relation to its current position and orientation on the y axis
* Orientation - the vechicle's projected orientation in radians in relation to its current orientation
* Velocity - calculated as the projected distance traveled since the last timestep
* Cross-track error - calculated as distance from the center of the car to the desired x location, as a cross section of the desired orientation
* Orientation error - calculated as the degree of difference from the car's projected orientation to the desired orientation in radians

### Actuators

Two actuator variables are maintained by the model as its means to act upon the world:

* Steering angle - required to stay between -25 degrees and 25 degrees
* Acceleration/Brakes - required to stay between -1 and 1 (which is the expectation of the simulator)

### Update Function

The specific code for the model's update function can be found in /src/MPC.cpp lines 115-122.

## Timestep Length and Elapsed Duration

The model projects forward 10 times at intervals of 0.15 seconds. Tuning of these parameters, n=10 and dt=0.15, was done through manual trial and error. Values tried were n=\[5, 10, 15, 20, 25\] and dt=\[0.5, 0.1, 0.15, 0.2, 0.25\].

Ultimately, n=10 was chosen on the basis of performing adequately while allowing for fast computations that didn't interupt driving.

The value dt=0.15 was chosen as a middle ground between too small (unable to anticipate sharp curves and susceptable to irratic behavior when not enough waypoints are near) and too large (prioritizing long term too much to drive safely short term).

### Polynomial Fitting and MPC Preprocessing

Prior to fitting a polynomial to the waypoints, the waypoints are reprojected from map space to vehicle space (In which x=0, y=0, and orientation=0 represents the car's current position, rather than the edge of a map). In addition, waypoints are modeled 100 ms into the future to account for latency, which will be discussed below.

### Model Predictive Control with Latency

In this simulator, there's a 100 millisecond delay introduced to force the mpc to handle more realistic conditions. This model primarily accounts for the latency by projecting 100 ms into the future, then treating the projected state as if it is the current state.

This introduced some challenges, namely that it can cause the car's orientation appear "jittery" when steering angles are allowed to move too quickly. Because the projection forward uses the same kinematic model as the model's update function, it's not perfect. This becomes quickly obvious if steering angles aren't kept relatively stable, and the "confused" car can proceed through a downward spiral that ends up off the side of the road.

I ended up handling this with two methods combined - tuning the cost function to keep steering angles more stable, and - the more forceful approach - by averaging the past steering value into the current one when accepting model outputs. This unfortunately eliminates most of the benefit to controlling the car, but at least leaves the waypoints mapped to the right position most of the time.

For next steps, I would like to use future actuations instead of just the returned current actuation as simulator inputs. As my model is right now, however, this approach is not working to regain the latency corrected model control output.

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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
