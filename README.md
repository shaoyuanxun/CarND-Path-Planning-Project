[image2]: images/behavior.png 
[image3]: images/spline.png 
# CarND-Path-Planning-Project
Self-Driving Car Engineer ND Program

## 1. Overview
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

<p align="center">
<img src="./video/path_planning.gif"]>
</p>

## 2. Motion Planning

The motion planner is responsible to decide whether the car should stay or change lane, and generate the map's coordinates (path) and speed that the car should operate. Our self-driving car achieves those goals by implementing a Finite State Machine (FSM) with 5 states, and minimizing the cost functions. The final path is generated using the spline math tool.

### 2.1 Finite State Machine
![alt text][image2]

* If the PLCL or PLCR are selected, the car prepares to change lane. The preparation checks if the car speed and the buffer space are safe to change lance. If there is enough buffer space to change lane, the FSM will transition to LCL/LCR states. The FSM returns to state KL as soon the lane change is over.



### 2.2 Cost Function

The cost function evaluates the cost for the FSM to change state. It evaluates different metrics trying to identify the next optimal state. Below, the metrics and how they are evaluated  are presented in detail:

* Lane Change Cost: The lane change adds a "comfort" cost.
* Buffer Cost: The buffer cost is propotional to the distance from the vehicle under question (in front or behind) diveded by the current speed. It helps the ego car to choose a lane with no traffic.
* Inefficiency Cost: This adds a cost of the vehicle speed deviated from the desired speed.
* Target Cost: The target cost evaluates the speed comparison between the ego car and the vehicle in front.
* Collision Cost: It strongly penalizes the states which the risk of collision.

### 2.3 Acceleration and Jerk control 
The requirements of this project state that the acceleration and jerk should not
exceed 10 m/s² and 50m/s³, respectively. In order to meet this requirement, the
car acceleration is increased or decreased by steps of 0.224 m/s².

### 2.4 Path Generation with Spline
![alt text][image3]

Spline is a piecewise "polynomial" parametric curve. They are popular for their simplicity of use and accuracy. The spline helps to define a smooth path for the car.

## 3. Dependency

This project requires the following packages to work:
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

## 4. Compiling and Running

The main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./path_planning
6. Run the Udacity Simulator (./term3_simulator)







