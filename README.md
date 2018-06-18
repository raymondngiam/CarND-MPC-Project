# Udacity Self Driving Car Nanodegree
## Model Predictive Control Project 

### Overview

This is a project for Udacity's Self Driving Car Nanodegree. The objective of this project is to implement Model Predictive Control in C++ to maneuver the vehicle around a race track in a simulated environment. The simulator will provide the desired future waypoints (from a path planning module), current global position of the vehicle and the velocity (mph). The cross track error CTE, appropriate steering angle and throttle command have to be determined at every measurement update.

---

### Video Demo



---

### Implementation Summary

**Vehicle Model**

The car has 4 states, ![](https://latex.codecogs.com/gif.latex?(x,y,v,\psi)^{T}) as illustrated in the figure below:

<img src="/images/vehicle-model.png" width="400">

It also has 2 actuator inputs, ![](https://latex.codecogs.com/gif.latex?(\delta,a)^{T}), where ![](https://latex.codecogs.com/gif.latex?\delta) is the steering angle, and ![](https://latex.codecogs.com/gif.latex?a) is the acceleration.

The next states are governed by the following dynamics:

![](https://latex.codecogs.com/gif.latex?x_{t+1}=x_{t}+v_{t}cos(psi_{t})\delta t)

![](https://latex.codecogs.com/gif.latex?)

![](https://latex.codecogs.com/gif.latex?)

![](https://latex.codecogs.com/gif.latex?)

**Cross Track Error (CTE) Computation**

Given a set of waypoints for a trajectory ![](https://latex.codecogs.com/gif.latex?%5Cleft%5C%7B%28x_%7Bw%2C1%7D%2Cy_%7Bw%2C1%7D%29%5E%7BT%7D%2C%28x_%7Bw%2C2%7D%2Cy_%7Bw%2C2%7D%29%5E%7BT%7D%2C...%2C%28x_%7Bw%2Cn%7D%2Cy_%7Bw%2Cn%7D%29%5E%7BT%7D%5Cright%5C%7D) and the vehicle pose ![](https://latex.codecogs.com/gif.latex?%28x_%7Bp%7D%2Cy_%7Bp%7D%2C%5Cpsi_%7Bp%7D%29%5ET) both in global coordinate system. It would be useful if we can transform the coordinate system to car reference in order to simplify the CTE computation. This is achieved by the following:

The distance of each waypoint relative to the vehicle position in global coordinate system is calculated. This distance vector is denoted as ![](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5E%7Bglobal%7D%5Ctextrm%7Bx%7D_%7Bcentered%7D%7D).

![](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5E%7Bglobal%7D%5Ctextrm%7Bx%7D_%7Bcentered%7D%7D%3D%5Cbegin%7Bpmatrix%7Dx_%7Bcentered%7D%5C%5Cy_%7Bcentered%7D%5Cend%7Bpmatrix%7D%3D%5Cbegin%7Bpmatrix%7Dx_%7Bw%2Ci%7D%5C%5Cy_%7Bw%2Ci%7D%5Cend%7Bpmatrix%7D-%5Cbegin%7Bpmatrix%7Dx_%7Bp%7D%5C%5Cy_%7Bp%7D%5Cend%7Bpmatrix%7D)

The waypoint in car coordinate system is then 

![](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5E%7Bcar%7D%5Ctextrm%7Bx%7D_%7Bw%2Ci%7D%7D%3D%5Cboldsymbol%7B%5E%7Bcar%7D%5Ctextrm%7BR%7D_%7Bglobal%7D%7D%5Ccdot%20%5Cboldsymbol%7B%5E%7Bglobal%7D%5Ctextrm%7Bx%7D_%7Bcentered%7D%7D)

where 

![](https://latex.codecogs.com/gif.latex?%5Cboldsymbol%7B%5E%7Bcar%7D%5Ctextrm%7BR%7D_%7Bglobal%7D%7D%3D%5Cbegin%7Bbmatrix%7Dcos%28-%5Cpsi_%7Bp%7D%29%26-sin%28-%5Cpsi_%7Bp%7D%29%5C%5Csin%28-%5Cpsi_%7Bp%7D%29%26cos%28-%5Cpsi_%7Bp%7D%29%5Cend%7Bbmatrix%7D)

The transformed waypoints are then used to fit a third order polynomial as follows:

![](https://latex.codecogs.com/gif.latex?f%28x%29%3Dc_%7B0%7D%2Bc_%7B1%7Dx%2Bc_%7B2%7Dx%5E%7B2%7D%2Bc_%7B3%7Dx%5E%7B2%7D)

The CTE at car coordinate ![](https://latex.codecogs.com/gif.latex?%28x%2Cy%29) is then defined as:

![](https://latex.codecogs.com/gif.latex?CTE%28x%2Cy%29%3Df%28x%29-y)

**Heading Error Computation**



**Vehicle Kinematic Model**

**Model Predictive Control Overview**

Model Predictive Control reframes the problem of following a trajectory as an optimization problem. It involves simulating different actuator inputs, prediction the resulting trajectory and selecting the trajectory with a minimum cost.

Imagine that we know our current state and the reference trajectory we want to follow. We optimize our actuator inputs at each step in time, in order to minimize the cost of our predicted trajectory.

Once we found the lowest cost trajectory, we implement the very first set of actuation commands. Then we throw away the rest of the trajectory we calculated. Instead of using the entire old trajectory we predicted, we take our new state and use that to calculate a new optimal trajectory.

The reason we do not just carry out the entire trajectory we calculated during the first pass is that our model is only approximate. Despite our best effort, it won't match the real world exactly. Once we performed our actuation commands, our trajectory might not be exactly the same as the trajectory we predicted. So, it's crucial that we constantly re-evaluate to find the optimal actuations.


**Latency Consideration**

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds.

This is a problem called "latency", and it's a difficult challenge for some controllers - like a PID controller - to overcome. But a Model Predictive Controller can adapt quite well because we can model this latency in the system.

PID Controller

PID controllers will calculate the error with respect to the present state, but the actuation will be performed when the vehicle is in a future (and likely different) state. This can sometimes lead to instability.

The PID controller could try to compute a control input based on a future error, but without a vehicle model it's unlikely this will be accurate.

Model Predictive Control

A contributing factor to latency is actuator dynamics. For example the time elapsed between when you command a steering angle to when that angle is actually achieved. This could easily be modeled by a simple dynamic system and incorporated into the vehicle model. One approach would be running a simulation using the vehicle model starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC.

Thus, MPC can deal with latency much more effectively, by explicitly taking it into account, than a PID controller.

