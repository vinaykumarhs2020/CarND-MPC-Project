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

With few trial and errors, I ended up using __N = 10__ and __dt = 0.15__. This combination gave a good performance on speed and helped to keep the vehicle at the center of the road even at the sharp turns.


### Latency and waypoints

- I used the travel time of the message to calculate the latency in realtime. `std::chrono::duration<double> latency =  end_time - start_time;` calculates the time spent between previous message generation and current time step.
- I use the trajectory information to fit a third order polynomial and use its coefficients to generate way points for x = [2,4,6 ... 20]. (The x domain is chosen to fit slightly longer line keeping the number of points to just 10)


### Additional observations

- Modelling without velocity didn't move the vehicle.
- Tried to model cross track error with a exponential loss function. This didn't help much
