# Training Sessions 4: Motion Control Systems and Tuning Strategies

Welcome to Training Sessions 4! In this session, we will delve into Motion Control Systems and Tuning Strategies for autonomous marine robotics. Below are the steps to set up and execute the training session.

### Step 1: Starting Docker Container
Begin by starting your Docker container.

#### Windows
  
  sh scripts/start_container.sh
  

#### Linux/MacOS

  ./scripts/start_container.sh

### Step 2: Running the Simulator

- Open a new terminal.
- Go to the ROS workspace for the training session 4:
  ```
  cd 34763-autonomous-marine-robotics/Training_Sessions/TS4_Motion_Control/ts4_ws
  ```
- Compile the workspace:
  ```
  catkin_make
  ```
- Source the compiled file:
  ```
  source devel/setup.bash
  ```
- Launch the simulator by running:
  ```
  roslaunch ts4_cascaded_pid start_pid_demo.launch gui:=false
  ```
This launch file initiates several processes including:
  - Spawning the robot in Gazebo.
  - Opening Rviz for visualization.
  - Running the PID controller (`position_control.launch`).
  - Displaying a comparison between the reference and ground truth of heading using `rqt_plot`.
  - Recording data to a bag file.
  
> Note: The Gazebo GUI is now turned off, you can turn it on by `gui:=true`

### Step 3: Running the ROS Node for Sending Setpoint Reference to the Robot
- Open another terminal by opening a new tab.
- Source the compiled file:
  ```
  source devel/setup.bash
  ```
- Execute the ROS node to publish velocity and position references to the robot:
  ```
  rosrun ts4_cascaded_pid ts4_command.py
  ```
  This script initially commands forward velocity and then introduces two heading changes (+/- 20 degrees).

- You should see the reference and the output response are plotted in `rqt_plot`.

- Ensure that the BAG file is generated and saved to `~/34763-autonomous-marine-robotics/Training_Sessions/TS4_Motion_Control/`.

### Step 4: Visualizing Data in MATLAB
- Open MATLAB and run `Training_session_4_visualize_bag.m`.
- Modify the `BAG_FILENAME` variable as needed.

![](/media/matlab_script_bag_file.png)

- This script visualizes the response of the position and velocity control.

![](/media/matlab_yaw_control.png)

Use "Data Tips" to analyze rise time, overshoot, and settling time.

### Step 5: Tuning PID Controller in Simulink

Open the MATLAB file `ts4_cascaded_PID_tuning.m`.

This file will 
  - Set the `Kp`, `Ki` & `Kd` parameters of the PID heading position and velocity controller 
  - Simulate a step response of the system (we use the identified heading model that you calculated in Training Session 2)
  - Plot the values of the step reference (`ref`) and the system output (`y`)

You task is now to play with the `Kp`, `Ki` & `Kd` parameters, untill you achieve a satisfactory response.

### Step 6: Deploying PID Constants in ROS Simulator
Update the PID constants for the position and velocity controllers in the following YAML files:
- Position controller: `~/34763-autonomous-marine-robotics/Training_Sessions/TS4_Motion_Control/ts4_ws/src/ts4_cascaded_pids/config/bluerov2/pos_pid_control.yaml`
- Velocity controller: `~/34763-autonomous-marine-robotics/Training_Sessions/TS4_Motion_Control/ts4_ws/src/ts4_cascaded_pids/config/bluerov2/vel_pid_control.yaml`

Moreover, there are saturation box after the PID controller boxes for limiting the control input. One of the reason why this saturation mechanism is usually used is because the input control of the robot is limited.

The lines that you need to change af the following:

```yaml
position_control/rot_p: <Kp_pos>
position_control/rot_i: <Kp_pos>
position_control/rot_d: <Kd_pos>
position_control/rot_sat: <sat_pos>
```

and

```yaml
velocity_control/angular_d: <Kp_vel>
velocity_control/angular_i: <Kp_vel>
velocity_control/angular_p: <Kp_vel>
velocity_control/angular_sat: <sat_vel>
```

Where you insert the parameters that you obtained in MATLAB.

![](/media/PID_constant.png)

### Step 7: Analyzing Results
Repeat steps 2 and 3, and engage in discussions based on the observed results.

Thank you for participating in Training Session 4! Feel free to reach out if you encounter any issues or have questions.


## Cascaded PID:

![](/media/PID_cascaded.png)

The PID controller implemented in the simulator is a cascaded controller for velocity and position control. The Matlab script and simulink are available (See: 'ts4_cascaded_PID_tuning.m' and 'Cascaded_PID_tuning.slx').


Tips for PID manual tuning:

![](/media/PID_table.png)
![](/media/PID_Compensation_Animated.gif)

