### Model Description:

This model uses 6 state variables (x,y,psi, v, cte, epsi) to represent vechicle parameters. Using cost functions defined in MPC.cpp, the model tries to minimize the cross track error and heading error while keeping in mind the costs for actuation and change in velocity and acceleration.


### Time length and Duration:

I tried to experiment with combination of `N` and `dt` parameters to improve the performance. Table below represents few of my observations:

| N | dt | Comment |
| --- | --- | ------ |
| 50 | 0.1 | Vehicle went out of the course every time |
| 10 | 0.25 |  goes on curb sometimes |
| 10 | 0.15 | Smooth |
| 25 | 0.25 | out of track sometimes|


### Additional observations

- Modelling without velocity didn't move the vehicle.
- Tried to model cross track error with a exponential loss function. This didn't help much
