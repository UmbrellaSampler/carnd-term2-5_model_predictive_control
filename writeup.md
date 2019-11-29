## Model Predictive Control (MPC) project
**Writeup Report**

The following rubric points will be addressed

* Student describes their model in detail. This includes the state, actuators and update equations.
* Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) 
values. Additionally the student details the previous values tried.
* A polynomial is fitted to waypoints.
  
  If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.
  
* The student implements Model Predictive Control that handles a 100 millisecond latency. 
Student provides details on how they deal with latency.

## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points

#### Model

***Student describes their model in detail. This includes the state, actuators and update equations.***

My projects implements a basic kinematic model that was introduced in the lessons. This model ignores the dynamics of 
the vehicle meaning that variables like tire forces, gravity and mass are not taken into consideration. 

The state comprises the following variables:

* x : x-coordinate
* y : y-coordinate
* ψ : orientation angle (psi)
* v : velocity
* cte : cross track error
* eψ : orientation error (epsi)

Furthermore, the following actuator variables are used

* a : acceleration
* δ : steering angle (delta)

The following update equations describe the kinematic model:

* x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(ψ<sub>t</sub>) * dt
* y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(ψ<sub>t</sub>) * dt
* ψ<sub>t+1</sub> = ψ<sub>t</sub> + v<sub>t</sub> / L<sub>f</sub> * δ<sub>t</sub> * dt
* v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> * dt
* cte<sub>t+1</sub> = y<sub>t</sub> − f(x<sub>t</sub>) + (v<sub>t</sub> * sin(eψ<sub>t</sub>)* dt)
* eψ<sub>t+1</sub> = ψ<sub>t</sub> − ψ<sub>des<sub>t</sub></sub> + ((v<sub>t</sub> / L<sub>f</sub>) * δ<sub>t</sub> * dt)

L<sub>f measures the distance between the center of mass of the vehicle and it's front axle.

#### Timestep Length and Elapsed Duration (N & dt)
***Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) 
values. Additionally the student details the previous values tried.***
    
I started using a relatively large number of N=25 and small dt = 0.05 (50ms). The controller performance was not optimal 
and lead to weird car behaviour. Successively, I decreased N and increase dt in several iterations. Finally I ended up 
with N = 10 and dt = 0.1 (100 ms) to get a good behavior.

#### Polynomial fitting

***A polynomial is fitted to waypoints.
If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.***

As a preprocessing step a coordinate transformation into the vehicle coordinate system was applied to the waypoints and 
the state.
Mainly for the following two reasons:

* Subsequent applied equations, e.g. kinematic equations, polynomial evaluations and error (cte, eψ) calculation get 
simplified under transformation due to (x = y = ψ = 0). 

* The simulator framework requires the waypoints and the predicted trajectory in vehicle frame coordinates in order to display it.
This comes out of the box if the cost optimization is done in vehicle coordinate system. 

#### Latency

***The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on 
how they deal with latency.***

The latency is handle in a preprocessing step before the MPC procedure. Once more, the kinematic update equations are
applied to the initial state in vehicle frame coordinates with dt=0.1 (100ms).