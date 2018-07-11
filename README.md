# ModelPredictiveControlSDCN

Model predictive Control project for Udacity Self Driving Car Nanodegree

### Overview

Model Predictive Control (MPC) involves simulating diffierent actuator inputs, predicting the resulting trajectory, and selecting the trajectory with minimum cost. If we know our current state and the reference trajectory we want to follow, we optimize our actuator inputs at each state in time in order to minimize the cost of our predicted trajectory. 
Once we found the trajectory with the lowest cost, we implement the first set of actuation commands. The the rest of the calculated trajectory is trown away, and instead of using that trajectory, we take our new state and calculate the new optimal trajectory. In that way, we are constantly calculating inputs over a future horizon. The reason we constantly calculate the new trajectory is because our model is only approximate, and the actual trajectory may not exactly match the predicted trajectory. So it it crucial to always re-evaluate and find the optimal actuation at every point in time. 
As shown in the following picture, MPC uses the initial state (left), the Model, the Constraints and the Cost function, and based on that it returns the vector of control inputs that minimize the given cost function. 

<img src="images/MPC_loop.png" width="700" alt="MPC loop equations" />

The cost function comprises of several components with the goal to:
- Be as close as possible to the reference state (zero cross track error, zero heading error, maintain the constant velocity)
- Minimize and smoothen control inputs such that we avoid large changes in the actuation between successive time steps
The constraints are set up such that the steering delta and the throttle delta are bounded with +/-25 degrees and +/-1 respectively.

### 


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
