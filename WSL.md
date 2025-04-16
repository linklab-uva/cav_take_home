# Windows Instructions:
We will use Windows Subsytem for Linux (WSL) to run Ubuntu 22.04 on Windows.

### Install WSL

Unless you have a reason not to, set Windows Terminal as the default terminal emulator. This can be done by opening up Windows Terminal, using the drop down menu and selecting "Settings". In the settings, scroll down to the "Startup" section and set the default terminal application to Windows Terminal.

1. Open PowerShell as an administrator and run the following command to install WSL 2 & Ubuntu:

```bash
wsl --install -d Ubuntu-22.04
```

2. Once this process is complete reboot your device. Upon reboot you will be prompted to create a new user account for WSL. This account can be different from your Windows account. Set a username and password for this account. Note that linux will not display the password as you type it in.

3. Within your Ubuntu installation run the following command:

```bash
cat /etc/os-release
```

This command will output the version of Ubuntu you are running. Check if you are running Ubuntu 22.04. If not you will need to upgrade your Ubuntu installation. This can be done by running the following commands:

```bash
sudo apt update && sudo apt full-upgrade
# restart Ubuntu
sudo do-release-upgrade
```

4. Open Powershell and run the following command:

```bash
wsl -l -v
```

This command will list all the WSL installations on your device. You should see a process with the name "Ubuntu" and a version number of 2. If you do not see this, you will need to set the default version of WSL to 2. This can be done by running the following command:

```bash
wsl --set-version Ubuntu 2
```

### Install ROS

The team utilizes ROS Humble. Open Ubuntu and run the following commands in your WSL environment:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt upgrade

sudo apt install ros-humble-desktop
```

1. For convenience, open your `.bashrc` file with the text editor of your choice and add the following line to the end of the file:

```bash
alias humble='source /opt/ros/humble/setup.bash'
```

You will want to run this command prior to running any ROS commands. This alias will automatically source the ROS environment for you by running `humble` in the terminal.

2. Then run the following command to prevent a future rviz error:

```bash
chmod 0700 /run/user/1000/
```

3. Then source your `.bashrc` file by running the following command:

```bash
source ~/.bashrc
```

and your ROS environment by running:

```bash
humble
```

4. Finally, run the following command to test if ROS is installed correctly:

```bash
ros2 doctor
```

5. Launch rviz by running the following command:

```bash
rviz2
```

After a few seconds, the rviz window should open.

### Dependencies

After installing ROS2, make sure to install the following dependencies.

```
sudo apt install ros-humble-camera-info-manager ros-humble-backward-ros ros-humble-ros2-socketcan ros-humble-image-proc ros-humble-camera-info-manager ros-humble-stereo-image-proc ros-humble-tracetools-image-pipeline ros-humble-udp-msgs ros-humble-rviz-common ros-humble-geographic-msgs ros-humble-angles ros-humble-osrf-testing-tools-cpp ros-humble-ament-cmake-google-benchmark ros-humble-random-numbers ros-humble-ros-testing ros-humble-point-cloud-msg-wrapper ros-humble-automotive-platform-msgs ros-humble-apex-test-tools ros-humble-pcl-ros ros-humble-joy-linux ros-humble-sensor-msgs-py ros-humble-gps-msgs ros-humble-nmea-msgs ros-humble-gps-tools ros-humble-can-msgs ros-humble-tf-transformations ros-humble-ament-cmake-nose ros-humble-rmw-cyclonedds-cpp ros-humble-rosbag2-storage-mcap ros-humble-camera-calibration ros-humble-foxglove-bridge -y
```

## Writing Code
After setup, you should clone this repository inside your WSL environment and follow the rest of the instructions in the main README. Make sure to put the your files somewhere inside `/home/username` or one of its sub-directories for the best experience with WSL.

There are multiple ways to edit files inside your WSL environment. The easiest is to use the [VSCode app and its extension for WSL](https://code.visualstudio.com/docs/remote/wsl). Follow the linked instructions to install and setup the extension. To edit code, simply run `code .` and this will open a Visual Studio Code window in the current folder.

## Visualization with Foxglove
In later steps, you will see "RViz" or "Foxglove". We recommend Foxglove for visualization.

Click "Open connection..." on Foxglove and use "ws://localhost:8765" as the websocket URL. Remember that in order to see anything in Foxglove, you should launch the Foxglove bridge with `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`.