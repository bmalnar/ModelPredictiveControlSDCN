# ModelPredictiveControlSDCN

Model predictive Control project for Udacity Self Driving Car Nanodegree

### Overview

Model Predictive Control (MPC) involves simulating diffierent actuator inputs, predicting the resulting trajectory, and selecting the trajectory with minimum cost. If we know our current state and the reference trajectory we want to follow, we optimize our actuator inputs at each state in time in order to minimize the cost of our predicted trajectory. 
Once we found the trajectory with the lowest cost, we implement the first set of actuation commands. The the rest of the calculated trajectory is trown away, and instead of using that trajectory, we take our new state and calculate the new optimal trajectory. In that way, we are constantly calculating inputs over a future horizon. The reason we constantly calculate the new trajectory is because our model is only approximate, and the actual trajectory may not exactly match the predicted trajectory. So it it crucial to always re-evaluate and find the optimal actuation at every point in time. 
As shown in the following picture, MPC uses the initial state (left), the Model, the Constraints and the Cost function, and based on that it returns the vector of control inputs that minimize the given cost function. 

<img src="images/MPC_loop.png" width="700" alt="MPC loop equations" />

_The image is taken from the Udacity course material_

The cost function comprises of several components with the goal to:
- Be as close as possible to the reference state (zero cross track error, zero heading error, maintain the constant velocity)
- Minimize and smoothen control inputs such that we avoid large changes in the actuation between successive time steps
The constraints are set up such that the steering delta and the throttle delta are bounded with +/-25 degrees and +/-1 respectively.

### Description of the MPC algorithm in the _src_ directory

- In the file _main.cpp_, in the function _onMessage(...)_, the program receives the telemetry message from the simulator with the vehicle state and the actuator values: waypoints, position, heading, velocity, steering value and throttle value. 
- The first step is to __transform the simulator waypoints__ to the vehicle coordinate system. This is done to simplify the polynomial fitting because the vehicle position is now at the origin of the coordinate system of the waypoints. 
```
for (unsigned int i = 0; i < num_waypoints; i++) {
   double delta_x = ptsx[i] - px;
   double delta_y = ptsy[i] - py;
   ptsx_transformed[i] = delta_x * cos_neg_psi - delta_y * sin_neg_psi;
   ptsy_transformed[i] = delta_x * sin_neg_psi + delta_y * cos_neg_psi;
}
```
- The next step is to fit the 3rd order polynomial to the transformed waypoints
```
auto poly_coeffs = polyfit(ptsx_transformed, ptsy_transformed, 3);
```
- At this point, we can calculate the initial state and the state after 100ms, which represents the actuator delay. We use these delayed state values in the function _mpc.Solve()_ to account for that actuator delay. 
```
double x_init    = 0.0;
double y_init    = 0.0;
double psi_init  = 0.0;
double cte_init  = poly_coeffs[0];
double epsi_init = -atan(poly_coeffs[1]);

double x_after = x_init + v * cos(psi_init) * actuator_delay_s;
double y_after = y_init + v * sin(psi_init) * actuator_delay_s;
double psi_after = psi_init - v * steering_angle * actuator_delay_s / mpc.Lf;
double v_after = v + throttle * actuator_delay_s;
double cte_after = cte_init + v * sin(epsi_init) * actuator_delay_s;
double epsi_after = epsi_init - v * atan(poly_coeffs[1]) * actuator_delay_s / mpc.Lf;

// State vector with the values after the actuator delay
Eigen::VectorXd state_after(6);
state_after << x_after, y_after, psi_after, v_after, cte_after, epsi_after;

// Calculate the MPC solution
auto vars = mpc.Solve(state_after, poly_coeffs);
```
- The function _mpc.Solve()_ returns the vector of actuation values, so we can now extract those and add them to the JSON message that we send back to the simulator. The simulator applies these values and thus closes one iteration in the loop. The next iteration starts with the simulator sending the next telemetry message to the program, which will trigger re-computation of the code above (transform waypoints, polyfit, solve)
```
double steer_value = vars[0];
double throttle_value = vars[1];
json msgJson;
// Divide by deg2rad(25) before you send the steering value back.
// Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
msgJson["steering_angle"] = steer_value / deg2rad(25);
msgJson["throttle"] = throttle_value;
```
- Note that the JSON message sent back from the program to the simulator also contains the MPC predicted trajectory (displayed with green color in the simulator), and the reference line based on the polynomial fit (yellow line in the simulator). These are used strictly for visualization, i.e. sanity checking that our MPC algorithm produces reasonable results. 

### Setting up the environment 
- The project is configured to compile with cmake and make. Please make sure that the following dependencies are met:
   - cmake version 3.5
   - make version 4.1 for Linux and Mac and 3.81 for Windows
   - gcc/g++ version 5.4
- Download the Udacity simulator from [here](https://github.com/udacity/self-driving-car-sim/releases/)
- Additional libraries need to be installed by running:
   - On Ubuntu, install-ubuntu.sh 
   - On Mac, install-mac.sh
   - On Windows, the recommended way is to run a virtual machine and use the install-ubuntu.sh script
- The project also uses libraries Ipopt, CppAD, and Eigen 
   - **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
   - [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
   
### More information
For even more information on the project structure, dependencies etc. please check original Udacity project [repository](https://github.com/udacity/CarND-MPC-Project)
