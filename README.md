# MPC Controller 

## Introduction:
The main goal of this project to implement a model predictive control or receding horizon controller in C++ using a kinematic model of the simulator car so that the car drives around the simulator track..!!

## Model:
Used a simple kinematic model for this project. The model just includes the following states of the vehicle: x & y coordinates of the car, velocity(v) of the car and the heading/orientation of the car (psi). A more complicated dynamic model can be used for MPC which takes into consideration the tire characteristic, drag mode, car geometry and actuator response into consideration to model the exact behavior of the car. While a dynamic model improves accuracy it certainaly introduces complexity. Here we dont have the necessary parameters and understanding of the car model to use a dynamic model and hence I worked with a simple kinematic model. 

The two main actuators used are:
steering angle --> which is in the range of [-25, 25] degrees. 

singular actuator (a) --> which signifies both throttle and braking i.e. accel and decel respectively.

Hence the final model uses the current state and current actuator commands to predict what the future state of the model is as follows:

![Eqns](https://github.com/aranga81/MPC-controller/blob/master/readme_images/Capture01.PNG)

Lf measures the distance between the front of the vehicle and the center of gravity of the car --> provided parameter..!!

Another important inclusion into model are the residual errors of the system. The errors are primarily cross track error (cte) --> difference b/w vehicles position and center of the lane or track. 
and Orientation error (epsi) --> desired orientation or heading of the car - current orientation of the car. 
![errors](https://github.com/aranga81/MPC-controller/blob/master/readme_images/Capture05.PNG)

The errors from the current time step along with the state and actuator inputs can be used to predict what the error for the next time step would be as follows:

![Eqns](https://github.com/aranga81/MPC-controller/blob/master/readme_images/Capture02.PNG)

![Eqns](https://github.com/aranga81/MPC-controller/blob/master/readme_images/Capture04.PNG)

## MPC Implementation & tuning:

The main goal here is to implement the MPC controller, tune the cost function and also consider the latency of the actuation into the controller implementation.

First the waypoints from the simulator are transformed into vehicles frame of reference. This makes sure the car is the the origin and the orientation angle is zero. It helps fit a 3rd order polynomical using the waypoints and the polyfit function. 

The value I choose for N was 10 and dt or the timestep is 0.1 i.e. 100ms similar to the latency of the actuators. Overall the look ahead of the MPC is for 1 second. I tries other values like 20/0.1 and 10/0.05 --> they mostly cause errors and significantly effect the cost function.

### Dealing with latency:
The delay in the actuators is about 100ms which is what the time step (dt) I choose for the controller. Now the state of the model in the curent timestep is due to the actuator commanded in the previous time step + the delay of dt(100 ms). To account for this latency I use the delayed actuator response in MPC.cpp file. 

One another idea that really helped me improve the control capability around corners is by including the the combination of velocity and delta while penalizing the cost function..!! [Slack and older students input..!! ]

The final reference velocity to safely navigate around the track was chosen to be 50 mph. 
The weights for penalizing - cte, epsi, [velocity - ref_velocity], delta, acceleration, change in delta and change in acceleration are all tuned and can be seen in MPC.cpp code.

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

