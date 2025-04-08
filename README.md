# Cavalier Autonomous Racing

Hello! The goal of these take-home tasks is to provide an introduction of some of the tools we use in our stack. These tools include C++ and ROS2, which you will learn about if you haven't already used them.

We want to see you become comfortable with writing code with the tools that run directly on our car and seeing some examples of real data from our car. You will visualize your results, and submit screen recordings of these visualizations as well as a GitHub repository with your code. We evaluate every submission on quality and completion of the tasks.

These tasks are definitely not easy, so we highly recommend joining our Slack. If there are **any** questions or ambiguities in the instructions, please send a message in `#all-cavalier-take-home-slack`. We encourage you to ask questions in this Slack channel so everyone can benefit from the answers. Furthermore, join `#office-hours` to stop by and get help in person.

Try your best and put in quality work – completing and understanding two tasks is better than ChatGPT-ing your way through all without understanding anything. You can definitely use generative AI to help, but don't rely on it too much; it can often be misleading for working with ROS2.

If you come across an unfamiliar word, first check the [TermsAndLinks](./TermsAndLinks.md) document.

With that said, let's dive in.

## Task 0: Install ROS2

Robot Operating System (ROS) is a framework that we use to develop our racing stack and interface with the car. The first step of this take home assignment is to install ROS2. **Make sure to install version humble.** These instructions are easiest to follow using Linux (Ubuntu 22.04). We recommend using Linux natively with dual boot, but you we also have instructions to set up Ubuntu 22.04 on MacOS or Windows.

[MacOS Instructions here](./MacOS.md)

[Windows (WSL) Instructions here](./WSL.md)

Regardless of how you get to Ubuntu 22.04, you should install ROS2 using the following instructions:

Linux: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Also install colcon: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

Also install foxglove https://foxglove.dev/ and `sudo apt install ros-humble-foxglove-bridge`

Also install mcap storage plugin `sudo apt install ros-humble-rosbag2-storage-mcap`

## Task 0.5: Learn Git

If you haven't worked with git before it is a version control tool that helps multiple contributors work on one project. While this won't be super critical for this take-home assignment, it will be when you are a crew member. If you haven't used git before, I highly recommend learning the basics. This is one introduction courrse I found https://learn.microsoft.com/en-us/training/modules/intro-to-git/, but feel free to use whatever. 

## Task 1: Complete ROS2 Tutorials

1. https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
2. https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html
3. https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html
4. https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html

If you are a mac user with docker you won't be able to run the turtle sim, but still read through the tutorials because understanding the publisher-subscriber architecture of ROS will be really important for the next task. 

## Task 2: Write code to compute metrics

There is a take_home package we have incluced with some stub code. Add to this ROS node some metrics, and create a separate topic for each metric. Download the example bag provided in the Slack.

To get started do the following:

1. Git clone this package.

2. Source ROS2 and build the template package and message dependencies. If you encounter any build issues after running the following, let us know.

```
cd ~/
git clone https://github.com/linklab-uva/cav_take_home.git
cd cav_take_home
source /opt/ros/humble/setup.bash
colcon build
```

3. To run the template node, source the install folder and run the executable.

```
source install/setup.bash
ros2 run take_home take_home_node
```

4. In a separate terminal, source your installation folder, and run the bag file.

```
source install/setup.bash
ros2 bag play -s mcap cavalier_take_home.mcap
```

5. In a separate terminal, run the following.

```
ros2 topic list
ros2 topic echo /metrics_output
```

The template node is subscribing to a message /vehicle/uva_odometry which contains the pose (position and orientation) and twist (linear and angular velocity) of our vehicle. There is a sample calculation that processes the pose and twist of the vehicle and outputs a value to the /metrics_output topic. It is your job to compute the meaningful metrics listed below, and publish them to new topics as we have shown in the example. 

### A. Wheel Slip Ratio

Wheel slip ratio is the ratio in the rotational speed of the wheel and the actual linear speed of the car. 

If you are interested in what this metric means check out: https://en.wikipedia.org/wiki/Slip_(vehicle_dynamics) (look at longitudinal slip section), but for this exercise we will give all the formulas and your job is to just implement them and show the ability to compute a metric from a given formula and publish it to a topic. 

Relevant topics:
- Vehicle Odometry (i.e. Pose and Twist) `/vehicle/uva_odometry`
- Wheel Speed: `/raptor_dbw_interface/wheel_speed_report` (kmph)
- Steering Wheel Angle `/raptor_dbw_interface/steering_extended_report` (deg)


The formula for calculating wheel slip ratio varies slightly between each of the four wheels. For the rear right wheel, we have the following formula:

$v_{x,rr} = v_x - 0.5 * \omega * w_r$

$\kappa_{rr} = (v_{rr}^w - v_{x,rr})/v_{x,rr}$

A similar calculation can be used for the rear left wheel:

$v_{x,rl} = v_x + 0.5 * \omega * w_r$

$\kappa_{rl} = (v_{rl}^w - v_{x,rl})/v_{x,rl}$

Here, $v_x$ refers to the car's longitudinal (forwards) linear speed in $m/s$. $\omega$ is the angular velocity (yaw rate) of the vehicle in $rad / s$. $w_r$ refers to the rear track width (the distance between left and right tire) in meters. $v^w$ refers to the wheel speed. $\kappa$ refers to the slip ratio of that wheel. $rr$ refers to the rear right wheel and $rl$ refers to the rear left wheel.

The front two wheels have similar formulas, but they have an extra transformation calculation since their orientation varies with the wheel angle. To get the wheel angle you can use the following topic `/raptor_dbw_interface/steering_extended_report`. There should be a field that describes the measured rotational angle in degrees of the steering motor in

`float32 primary_steering_angle_fbk`.

To get the wheel angle, use this value and divide by a steering ratio $15.0$. Use this wheel angle as $\delta$ in the following transformations.


For the front right wheel the full formula is:

$v_{x,fr} = v_x - 0.5 * \omega * w_f$

$v_{y,fr} = v_y + \omega * l_f$

$v_{x,fr}^\delta = cos(\delta) * v_{x,fr} - sin(\delta) * v_{y,fr}$

$\kappa_{fr} = (v_{fr}^w - v_{x,fr}^\delta)/v_{x,fr}^\delta$

For the front left wheel the formula is:

$v_{x,fl} = v_x + 0.5 * \omega * w_f$

$v_{y,fl} = v_y + \omega * l_f$

$v_{x,fl}^\delta = cos(\delta) * v_{x,fl} - sin(\delta) * v_{y,fl}$

$\kappa_{fl} = (v_{fl}^w - v_{x,fl}^\delta)/v_{x,fl}^\delta$

Here, the same variables from before are used, with the addition of $\delta$, which refers to steering angle (in radians), $w_f$, which refers to the front track width (in meters), $v_y$, which refers to the car's lateral (tangential) linear speed, in $m/s$, and $l_f$, which is to the longitudinal distance from the COG of the car to the front wheels (in meters). Here, $fr$ refers to the front right wheel and $fl$ refers to the front left wheel.

Use the following values for the constants:
- **$w_f$** = 1.638
- **$w_r$** = 1.523
- **$l_f$** = 1.7238

We want to see the following values published on new topics:
- `slip/long/rr`:  $\kappa_{rr}$
- `slip/long/rl`:  $\kappa_{rl}$
- `slip/long/fl`:  $\kappa_{fl}$
- `slip/long/fr`:  $\kappa_{fr}$

### B. Jitter in IMU data

Our car runs with two GNSS sensors from novatel which both report IMU measurements. Ideally, we would like these sensors to publish at a consistent ∆t (ie a fixed rate). Jitter is defiend as the variance of the ∆t between consecutive measurements. Compute the jitter for the top IMU report it. Compute this metric using a sliding window and consider the last `1s` of data in this sliding window. Our goal is to see your ability to write logic that can handle messages across time intervals.

Relevant toipcs: 
- Top IMU: `/novatel_top/rawimu`

We want to see the following values published on new topics:
- `imu_top/jitter`

### C. Lap time

How long does it take in seconds for the vehicle to complete 1 lap.

Relevant topics:
- Vehicle Odometry (i.e. Pose and Twist) `/vehicle/uva_odometry`
- Curvilinear Distance (Forward progress along the track in meters) `curvilinear_distance`

We want to see the following values published on new topics:
- `lap_time`

## Task 3: Run your code and visualize the results

1. Please refer to the pinned message in the shared slack to get access to a ROS2 bag with data from our time trial run. If you are using MacOS, refer to the MacOS instructions for how to get this file into Docker.

2. Launch foxglove and launch the foxglove bridge `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`. Make sure you source your install before launching the bridge (see below about sourcing).

3. Foxglove supports making some really useful panels with plots, 3D visualization position / sensor data, and more. We have attached a panel to start with (`panel.json`) which just creates an empty plot and a 3D visualization that shows the LiDAR scans. An important part of working on the stack invovles adding to foxglove panels to visualize and plot your data, so adjust the given panel to include all of the metrics A-C above. 

4. Screen record the foxglove app and play the entire bag, so we can see the metrics you computed. 

For an example of how to launch foxglove and create a plot to visualize your metric in realtime, you can refer to [this example video](https://drive.google.com/file/d/1MFAJuCiuM2APs_FErlB4Ts95pXXj2gts/view?usp=sharing)

## Task 4: Make a PR for your fork back into the main repository

Please upload your screen recording and share the link to it as a part of the PR. 

**To get credit for completing any of the metrics in task 3, this step is mandatory**

## Getting Started + Tips Tricks

### Clone this repo into your home directory

```{bash}
git clone git@github.com:linklab-uva/cav_take_home.git
```

### Sourcing
Whenever you run anything, it is critical that you source your workspace. This allows you to work with our custom ROS2 message types and run the packages you've built. 

```{bash}
source ~/cav_take_home/install/setup.sh
```
