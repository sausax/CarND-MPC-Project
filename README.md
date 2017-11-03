# CarND-Controls-MPC

This is a solution to Model Predictive Control programming assignment for Self-Driving Car Engineer Nanodegree Program


## Model details

The model uses following equation to calculate the state for next timestamp based on the state from previous timestamp. 

x_t_1 = x_t + v_t * cos(psi_t) * dt
y_t_1 = y_t + v_t * sin(psi_t) * dt
psi_t_1 = psi_t + (v_t/L_f) * delta_t * dt
v_t_1 = v_t + a_t * dt
cte_t_1 = f(x_t) - y_t + (v_t * sin(epsi_t) * dt)
epsi_t_1 = psi_t - psid_t + ((v_t/L_f) * delta_t * dt)

## Timestep length and duration

By trial and error method I came up with timestep lenght N=10 and duration dt=0.1. Using these values optimizer is considering time duration of 1 sec. Few other values that I tried were 25/0.05, 20/0.05, 20/0.1 and 15/0.1. These values were not predicting a smooth path for the car.


## Polynomial Fitting and Waypoint preprocessing

I preprocess the waypoint received from simulator transform them for car's perspective (main.cpp 110-115). After that I fit the points to a 3 dimensional polynomial (main.cpp line 120).

## MPC with latency 

To deal with 100ms latency in actuators I did two main things

* Use a/delta values from 2 time step back when calculating new state. (MPC.cpp 78 - 81)
* Add extra penalty for steer and acceleration to punish sharp change. 
