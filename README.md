# Cavalier Autonomous Racing

Hello! The goal of this is to give you an introduction to what it is like to work with C++ / ROS2 and data we record during every run. To join our team, we want to see both completion of these tasks and high quality work.

What is expected (in decreasing order of importance)
1. Testing your code (i.e. visualize the data as plots in Foxglove, create informative layouts, etc.)
2. Asking good questions in Slack (i.e. questions that ChatGPT can't answer with one prompt)
3. Starting work before the deadline + seeing consistent progress
4. High quality code (i.e. clean code, good variable names, etc.)
5. Completing all four tasks

## Task 1: Create a C++ ROS2 package inside of src/

Use this tutorial as an example: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

You can also reference the `trajectory_propagator` package for a example of how to write nodes (NOTE: this package won't build here because we haven't included all the necessary dependencies)

## Task 2



## Task 3



## Task 4



## Getting Started + Tips Tricks

### Building

In ROS 2 we use `colcon` for building all of our packages. Always build your packages from `~/cav_take_home`

### Join the Slack to ask questions

(must join with @virginia.edu email)
https://join.slack.com/t/cavaliertakehomeslack/shared_invite/zt-309wg9j29-RbvdsdKOC7ls_C34z7jGNw

### Clone this repo into your home directory

```{bash}
git clone <> 
```

### Building All Relevant Messages

```{bash}
colcon build --packages-up-to deep_orange_msgs uva_iac_msgs localization_msgs raptor_dbw_msgs
```

### Sourcing
Whenever you run anything, it is critical that you source your workspace. This allows you to work with our custom ROS2 message types and run the packages you've built. 

```{bash}
source ~/cav_take_home/install/setup.sh
```

