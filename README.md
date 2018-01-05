# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## The Model

The MPC uses the bicycle model, which assumes both the front and back wheels work together, which makes the car into an extended bicycle. 
The states of the car are the following: `x` (x coordinate), `y` (y coordinate), `psi` (yaw), `v` (speed), `cte` (error from the planned trajectory), `epsi` (error from the planned yaw).
There are two actuators, or controlling elements with constraints, `delta` (steering angle) and `a` (throttle). Steering angle is limited to [-25,25] degrees while the throttle is limited to [-1,1] range of the similator. 
Below are the update equations:

`x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt`

## Time Step Length && Elasped Duration

Since the delay time is about 100 ms, I set the elasped duration to be 50 ms, to calculate twice before sending out one control. This gives flexibility for calculation accuracy, but is not too dense. I set the time step length to be 15, so that the car calculates 3/4 of a second of its planned trajectory.

## MPC preprocessing

Before inputing the state for the MPC, I first transformed the coordinates of the planned trajectory from world coordinate to vechicle coordinates through a rotational transformation. The rotational matrix used is `{{cos(psi),-sin(psi)},{sin(psi),cos(psi)}}`;

## Latency
I delt with the latency by putting that into the MPC model, that is, to have variable constraints during the optimization process. 

For time step t, if `t < latency time / dt = 2`, then I set `delta[t]` to be `previous delta` and `a[t]` to be `previous a`. When calculating the variables that minimizes the cost, I discard the first 2 values of actuations, which are expected to be the previous actuations. Instead, I return `delta[2]` and `a[2]`, which is the third actuation calculated. In that way, the next instruction I send will be the exact ones I need after the latency, that is, 2*dt. In that way, the controls are always up to date. 

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

