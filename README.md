# Udacity Self Driving Car Nanodegree
## Model Predictive Control Project 

### Overview

This is a project for Udacity's Self Driving Car Nanodegree. The objective of this project is to implement Model Predictive Control in C++ to maneuver the vehicle around a race track in a simulated environment. The simulator will provide a set of desired future waypoints, current global position of the vehicle and the velocity (mph). The appropriate steering angle and throttle command have to be determined at every measurement update.

---

### Video Demo

<img src="/images/demo.gif" width="600">

The yellow is a polynomial fitted to waypoints and the green line represents the x and y coordinates of the MPC trajectory.

---

### Implementation Summary

**Vehicle Model**

The car has 4 states, ![](https://latex.codecogs.com/gif.latex?(x,y,v,\psi)^{T}) as illustrated in the figure below:

<img src="/images/vehicle-model.png" width="400">

It also has 2 actuator inputs, ![](https://latex.codecogs.com/gif.latex?(\delta,a)^{T}), where ![](https://latex.codecogs.com/gif.latex?\delta) is the steering angle, and ![](https://latex.codecogs.com/gif.latex?a) is the acceleration.

The states at the next time step are governed by the following dynamics:

![](https://latex.codecogs.com/gif.latex?x_%7Bt%2B1%7D%3Dx_%7Bt%7D%2Bv_%7Bt%7Dcos%28%5Cpsi_%7Bt%7D%29%5CDelta%20t)

![](https://latex.codecogs.com/gif.latex?y_%7Bt%2B1%7D%3Dy_%7Bt%7D%2Bv_%7Bt%7Dsin%28%5Cpsi_%7Bt%7D%29%5CDelta%20t)

![](https://latex.codecogs.com/gif.latex?%5Cpsi_%7Bt%2B1%7D%3D%5Cpsi_%7Bt%7D%2B%5Cfrac%7Bv_%7Bt%7D%7D%7BL_%7Bf%7D%7D%5Cdelta_%7Bt%7D%5CDelta%20t)

![](https://latex.codecogs.com/gif.latex?v_%7Bt%2B1%7D%3Dv_%7Bt%7D%2Ba_%7Bt%7D%5CDelta%20t)

where ![](https://latex.codecogs.com/gif.latex?L_{f}) measures the distance between the center of mass of the vehicle and it's front axle. The larger the vehicle, the slower the turn rate.

The value of ![](https://latex.codecogs.com/gif.latex?L_{f}) can be determined empirically. Using the vehicle in simulated environment, we drive at constant steering angle and velocity. We incrementally adjust ![](https://latex.codecogs.com/gif.latex?L_{f}) such that the trajectory produced is a circle, with all other variables held constant. The figure below shows such trajectory with a constant ![](https://latex.codecogs.com/gif.latex?\delta) value of 1 degree and ![](https://latex.codecogs.com/gif.latex?L_{f}) of 2.67.

<img src="/images/lf.png" width="400">

**Trajectory Polynomial Fitting**

Given a set of waypoints for a trajectory ![](https://latex.codecogs.com/gif.latex?%5Cleft%5C%7B%28x_%7Bw%2C1%7D%2Cy_%7Bw%2C1%7D%29%5E%7BT%7D%2C%28x_%7Bw%2C2%7D%2Cy_%7Bw%2C2%7D%29%5E%7BT%7D%2C...%2C%28x_%7Bw%2Cn%7D%2Cy_%7Bw%2Cn%7D%29%5E%7BT%7D%5Cright%5C%7D) and the vehicle pose ![](https://latex.codecogs.com/gif.latex?%28x_%7Bp%7D%2Cy_%7Bp%7D%2C%5Cpsi_%7Bp%7D%29%5ET) both in global coordinate system. It would be useful if we can transform the coordinate system to car reference in order to simplify the subsequent computations. This is achieved by the following:

The distance of each waypoint relative to the vehicle position in global coordinate system is calculated. This distance vector is denoted as ![](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5E%7Bglobal%7D%5Ctextrm%7Bx%7D_%7Bcentered%7D%7D).

![](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5E%7Bglobal%7D%5Ctextrm%7Bx%7D_%7Bcentered%7D%7D%3D%5Cbegin%7Bpmatrix%7Dx_%7Bcentered%7D%5C%5Cy_%7Bcentered%7D%5Cend%7Bpmatrix%7D%3D%5Cbegin%7Bpmatrix%7Dx_%7Bw%2Ci%7D%5C%5Cy_%7Bw%2Ci%7D%5Cend%7Bpmatrix%7D-%5Cbegin%7Bpmatrix%7Dx_%7Bp%7D%5C%5Cy_%7Bp%7D%5Cend%7Bpmatrix%7D)

The waypoint in car coordinate system is then 

![](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5E%7Bcar%7D%5Ctextrm%7Bx%7D_%7Bw%2Ci%7D%7D%3D%5Cboldsymbol%7B%5E%7Bcar%7D%5Ctextrm%7BR%7D_%7Bglobal%7D%7D%5Ccdot%20%5Cboldsymbol%7B%5E%7Bglobal%7D%5Ctextrm%7Bx%7D_%7Bcentered%7D%7D)

where 

![](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5E%7Bcar%7D%5Ctextrm%7BR%7D_%7Bglobal%7D%7D%3D%5Cbegin%7Bbmatrix%7Dcos%28-%5Cpsi_%7Bp%7D%29%26-sin%28-%5Cpsi_%7Bp%7D%29%5C%5Csin%28-%5Cpsi_%7Bp%7D%29%26cos%28-%5Cpsi_%7Bp%7D%29%5Cend%7Bbmatrix%7D)

The transformed waypoints are then used to fit a third order polynomial as follows:

![](https://latex.codecogs.com/gif.latex?f%28x%29%3Dc_%7B0%7D%2Bc_%7B1%7Dx%2Bc_%7B2%7Dx%5E%7B2%7D%2Bc_%7B3%7Dx%5E%7B2%7D)

**Heading Error Computation**

Heading error, ![](https://latex.codecogs.com/gif.latex?e%5Cpsi) is the desired orientation subtracted from the current orientation

![](https://latex.codecogs.com/gif.latex?e%5Cpsi_%7Bt%7D%3D%5Cpsi_%7Bt%7D-%5Cpsi%20des_%7Bt%7D)

![](https://latex.codecogs.com/gif.latex?%5Cpsi%20des_%7Bt%7D) can be calculated as the tangential angle of the polynomial ![](https://latex.codecogs.com/gif.latex?f) evaluated at ![](https://latex.codecogs.com/gif.latex?x_{t})

![](https://latex.codecogs.com/gif.latex?%5Cpsi%20des_%7Bt%7D%3D%5Carctan%28f%27%28x_t%29%29)

The update rule for heading error is as follows:

![](https://latex.codecogs.com/gif.latex?e%5Cpsi_%7Bt%2B1%7D%3De%5Cpsi_%7Bt%7D%2B%5Cfrac%7Bv_%7Bt%7D%7D%7BL_%7Bf%7D%7D%5Cdelta_%7Bt%7D%5CDelta%20t)

![](https://latex.codecogs.com/gif.latex?e%5Cpsi_%7Bt%2B1%7D%3D%5Cpsi_%7Bt%7D-%5Cpsi%20des_%7Bt%7D%2B%5Cfrac%7Bv_%7Bt%7D%7D%7BL_%7Bf%7D%7D%5Cdelta_%7Bt%7D%5CDelta%20t)

**Cross Track Error (CTE) Computation**

The CTE at car coordinate ![](https://latex.codecogs.com/gif.latex?%28x%2Cy%29) is then defined as:

![](https://latex.codecogs.com/gif.latex?cte%28x%2Cy%29%3Df%28x%29-y)

CTE at the next time step is defined as

![](https://latex.codecogs.com/gif.latex?cte_%7Bt%2B1%7D%3Dcte_%7Bt%7D%2Bv_%7Bt%7Dsin%28e%5Cpsi%29%5CDelta%20t)

![](https://latex.codecogs.com/gif.latex?cte_%7Bt%2B1%7D%3Df%28x_t%29-y_t%2Bv_%7Bt%7Dsin%28e%5Cpsi%29%5CDelta%20t)

**Full Vehicle Kinematic Model**

The crucial equations for the full kinematic model are summarized below:

![](https://latex.codecogs.com/gif.latex?x_%7Bt%2B1%7D%3Dx_%7Bt%7D%2Bv_%7Bt%7Dcos%28%5Cpsi_%7Bt%7D%29%5CDelta%20t)

![](https://latex.codecogs.com/gif.latex?y_%7Bt%2B1%7D%3Dy_%7Bt%7D%2Bv_%7Bt%7Dsin%28%5Cpsi_%7Bt%7D%29%5CDelta%20t)

![](https://latex.codecogs.com/gif.latex?%5Cpsi_%7Bt%2B1%7D%3D%5Cpsi_%7Bt%7D%2B%5Cfrac%7Bv_%7Bt%7D%7D%7BL_%7Bf%7D%7D%5Cdelta_%7Bt%7D%5CDelta%20t)

![](https://latex.codecogs.com/gif.latex?v_%7Bt%2B1%7D%3Dv_%7Bt%7D%2Ba_%7Bt%7D%5CDelta%20t)

![](https://latex.codecogs.com/gif.latex?cte_%7Bt%2B1%7D%3Df%28x_t%29-y_t%2Bv_%7Bt%7Dsin%28e%5Cpsi%29%5CDelta%20t)

![](https://latex.codecogs.com/gif.latex?e%5Cpsi_%7Bt%2B1%7D%3D%5Cpsi_%7Bt%7D-%5Cpsi%20des_%7Bt%7D%2B%5Cfrac%7Bv_%7Bt%7D%7D%7BL_%7Bf%7D%7D%5Cdelta_%7Bt%7D%5CDelta%20t)

**Constraints**

The constraints in the simulation environment are as follows:

![](https://latex.codecogs.com/gif.latex?\delta\in[-25^{\circ},25^{\circ}])

![](https://latex.codecogs.com/gif.latex?a\in[-1,1])

The steering angle is limited to be between -25 degree to +25 degree, and the acceleration (or throttle) is limit to be between -1 and +1.

**Model Predictive Control**

Model Predictive Control reframes the problem of following a trajectory as an optimization problem. It involves simulating different actuator inputs, prediction the resulting trajectory and selecting the trajectory with a minimum cost.

Imagine that we know our current state and the reference trajectory we want to follow. We first define the number of time step, ![](https://latex.codecogs.com/gif.latex?N) and the timestep duration, ![](https://latex.codecogs.com/gif.latex?dt).

<img src="/images/MPC-N-dt.png" width="600">

In the figure above, the blue line is the reference trajectory and the red line the trajectory computed by Model Predictive Control. In this example the horizon has 7 steps, ![](https://latex.codecogs.com/gif.latex?N), and the space in between white pebbles signifies the time elapsed, ![](https://latex.codecogs.com/gif.latex?dt).

The prediction horizon is the duration over which future predictions are made. We’ll refer to this as ![](https://latex.codecogs.com/gif.latex?T). ![](https://latex.codecogs.com/gif.latex?T) is the product of ![](https://latex.codecogs.com/gif.latex?N) and ![](https://latex.codecogs.com/gif.latex?dt).

In the case of driving a car, ![](https://latex.codecogs.com/gif.latex?T) should be a few seconds, at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future.

The choice of ![](https://latex.codecogs.com/gif.latex?dt) is also crucial here. MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations. Larger values of ![](https://latex.codecogs.com/gif.latex?dt) result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. This is sometimes called "discretization error".

In this project, the ![](https://latex.codecogs.com/gif.latex?N) and ![](https://latex.codecogs.com/gif.latex?dt) parameters are set to be 10 and 0.1 respectively. Thus, the effective prediction horizon, ![](https://latex.codecogs.com/gif.latex?T) here is `10*0.1=1 second`. 

We optimize our actuator inputs at each step in time, in order to minimize the cost of our predicted trajectory. The cost function used is as shown below:

![](https://latex.codecogs.com/gif.latex?J%3D%5Csum_%7Bi%3D1%7D%5E%7BN%7Dw_%7Bcte%7D%28cte_t-cte_%7Bref%7D%29%5E2%2Bw_%7Be%5Cpsi%7D%28e%5Cpsi_t-e%5Cpsi_%7Bref%7D%29%5E2%2Bw_%7Bv%7D%28v_t-v_%7Bref%7D%29%5E2%2Bw_%7B%5Cdelta%7D%5Cdelta_t%5E2%2Bw_%7Ba%7Da_t%5E2%2Bw_%7B%5Cdelta-rate%7D%28%5Cdelta_%7Bt%2B1%7D-%5Cdelta_%7Bt%7D%29%5E2%2Bw_%7Ba-rate%7D%28a_%7Bt%2B1%7D-a_%7Bt%7D%29%5E2)

Here, the reference cross track error (![](https://latex.codecogs.com/gif.latex?cte_{ref})), orientation error (![](https://latex.codecogs.com/gif.latex?e\psi_{ref})) and speed (![](https://latex.codecogs.com/gif.latex?v_{ref})) are set as follows:

|        | Value   |
| ------:|-------:|
| ![](https://latex.codecogs.com/gif.latex?cte_{ref})      	| 0 |
| ![](https://latex.codecogs.com/gif.latex?e\psi_{ref})     | 0 |
| ![](https://latex.codecogs.com/gif.latex?v_{ref})	 		| 100 |

The weights for each of the cost terms are fine tuned manually and the final configuration is as shown below:

|        | Value   |
| ------:|-------:|
| ![](https://latex.codecogs.com/gif.latex?w_{cte})      	| 15 |
| ![](https://latex.codecogs.com/gif.latex?w_{e\psi})     	| 15 |
| ![](https://latex.codecogs.com/gif.latex?w_{v})	 		| 0.02 |
| ![](https://latex.codecogs.com/gif.latex?w_{\delta})	 	| 20000 |
| ![](https://latex.codecogs.com/gif.latex?w_{a})	 		| 0 |
| ![](https://latex.codecogs.com/gif.latex?w_{\delta-rate})	| 250000 |
| ![](https://latex.codecogs.com/gif.latex?w_{a-rate})	 	| 0 |

Once we found the lowest cost trajectory, we implement the very first set of actuation commands. Then we throw away the rest of the trajectory we calculated. Instead of using the entire old trajectory we predicted, we take our new state and use that to calculate a new optimal trajectory.

The figure below shows the trajectory computed:

<img src="/images/trajectory-0.png" width="600">

The car then takes only the first set of actuation commands toward the next execution time step.

<img src="/images/trajectory-1.png" width="600">

The reason we do not just carry out the entire trajectory we calculated during the first pass is that our model is only approximate. Despite our best effort, it won't match the real world exactly. Once we performed our actuation commands, our trajectory might not be exactly the same as the trajectory we predicted. So, it's crucial that we constantly re-evaluate to find the optimal actuations.


**Latency Consideration**

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds.

This is a problem called "latency", and it's a difficult challenge for some controllers - like a PID controller - to overcome. But a Model Predictive Controller can adapt quite well because we can model this latency in the system.

PID controllers will calculate the error with respect to the present state, but the actuation will be performed when the vehicle is in a future (and likely different) state. This can sometimes lead to instability. The PID controller could try to compute a control input based on a future error, but without a vehicle model it's unlikely this will be accurate.

MPC can deal with latency much more effectively, by explicitly taking it into account, than a PID controller.

Here we run a simulation using the vehicle model starting from the current state for the duration of the latency (100 ms). We then set the resulting state from the simulation as the new initial state for MPC.

---

### Result

The following three graphs show the output for the three states we are interested in controlling, namely ![](https://latex.codecogs.com/gif.latex?cte) with ![](https://latex.codecogs.com/gif.latex?cte_{ref}=0), ![](https://latex.codecogs.com/gif.latex?e\psi) with ![](https://latex.codecogs.com/gif.latex?e\psi_{ref}=0), and finally ![](https://latex.codecogs.com/gif.latex?v) with ![](https://latex.codecogs.com/gif.latex?v_{ref}=100).

<img src="/images/cte.png" width="600">

<img src="/images/epsi.png" width="600">

<img src="/images/speed.png" width="600">

The next two graphs shows the two actuator inputs, ![](https://latex.codecogs.com/gif.latex?\delta) and ![](https://latex.codecogs.com/gif.latex?a) which are generated by the MPC controller.

<img src="/images/steering_angle.png" width="600">

<img src="/images/throttle.png" width="600">

---

### Installation

1. Download the Udacity Self Driving Car Nanodegree simulator from [here](https://github.com/udacity/self-driving-car-sim/releases).

2. Set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) by running the shell script in the project top directory:

```
$ install-ubuntu.sh
```

**Other Important Dependencies**

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* **Ipopt and CppAD:** Please refer to [this document](./install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

---

### How to run

The main program can be built and run by doing the following from the project top directory.

1. mkdir build

2. cd build

3. cmake ..

4. make

5. ./mpc

Open the simulator, select `Project 5: MPC Controller`

