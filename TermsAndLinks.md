colcon: A build tool, similar to `make` for C. colcon is used to build ROS2 packages

Foxglove: A tool for visualizing ROS2 nodes and topics. It can be used to visualize these topics in real-time, i.e. while the code is running. It can also be used to play a ros2 bag. There are a TON of features in Foxglove. We will be working mostly with [plots](https://docs.foxglove.dev/docs/visualization/panels/plot).


git: A version control system for code. A "repository" in git is like a project folder that you can share with collaborators working on the same code. GitHub is a place where you can host repositories, and it's where you will start with this take-home.

ROS2: [ROS2](https://docs.ros.org/en/humble/_downloads/2a9c64e08982f3709e23d20e5dc9f294/ros2-brochure-ltr-web.pdf) is a framework for developing software for robots. A whole project is composed of individual modules called [ROS2 nodes](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html), which communicate with each other by sending messages as [ROS2 topics](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).

ROS2 bag: A [ROS2 bag](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) is a way to record and play back data from a robot. In these tasks, you will use a ros2 bag containing real data from our cars.


RVIZ2: A tool for visualizing ROS2 topics. It's pretty good, but in most cases, Foxglove is easier to use. Wherever you see "RVIZ2" in the instructions, feel free to use either RVIZ2 or Foxglove.