
# Model Predictive Control
Model Predictive Control considers the task of following a trajectory as an optimization problem in which the solution is the path the car should take. The idea is to simulate different actuator inputs (steering, acceleration and braking) and predict a resulting trajectory by selecting the one with the minimum cost. The car follows that trajectory and gets new input to calculate a new set of trajectories to optimize. The model utilizes the called “receding horizon controller” which performs a trajectory recalculation for every new state, since the defined trajectory is just an approximation.

![Image of MPC](images/mpc.png)

## The Model
I used a global kinematic model, which is a simplification of a dynamic model that ignores tire forces, gravity and mass.
The state model is represented by the vehicles position, orientation angle (in radians) and velocity.

![Image of The Model](images/themodel.png)

A cross track error (distance of vehicle from trajectory) and an orientation error (difference of vehicle orientation and trajectory orientation) were also included in the state model. Two actuators were used, delta — to represent the steering angle (normalised to [-1,1]) and a — for acceleration corresponding to a throttle, with negative values for braking.

The simulator passes via a socket, ptsx & ptsy of six waypoints (5 in front, 1 near the vehicle), the vehicle x,y map position, orientation and speed (mph).This data after being transformed into the vehicle map space, with new cross track error and orientation error calculated, is then passed into the MPC (Model Predictive Control) solve routine. It returns, the two new actuator values, with steering and acceleration (i.e. throttle) and the MPC predicted path (plotted in green in the simulator).

Constraint costs were applied to help the optimiser select an optimal update. Emphasis was placed on minimising orientation error and actuations, in particular steering (to keep the lines smooth).

## Timestep Length and Elapsed Duration (N & dt)
The MPC optimiser has two variables to represent the horizon into the future to predict actuator changes. They are determined by N (Number of timesteps) and dt (timestep duration) where T (time) = N * dt.

To help tune these settings, I copied the mpc_to_line project quiz, to a new project mpc_to_waypoint, and modified it to represent the initial state model to be used with the Udacity simulator. I was able to get good results looking out 3 seconds, with N = 15 and dt = 0.2. The following output are plots of 50 iterations from the initial vehicle state:



## Polynomial Fitting and MPC Preprocessing




## Model Predictive Control with Latency



## Final simulation





## Comments 
