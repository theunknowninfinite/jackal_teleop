# ENPM690 Homework 3
This folder contains the work done for Homework 3.

This repo contains the dockerfile required to build and the package as a containerized app.
The required steps to run it are given below.

# Jackal Teleop and Move Base

This package contains the necessary launch files to run the teleop and move base based AMCL navigation functionalities for Jackal using ROS 1 Noetic.

## Prerequisites

Before running the launch files, make sure you have the following prerequisites installed:

- ROS 1 Noetic: [Installation Guide](http://wiki.ros.org/noetic/Installation)
- Ubuntu 20.04
- xterm

# Part One
## Teleoperation

1. Install the required packages

     ```bash
    sudo apt-get install xterm
    rosdep init # if not done otherwise DO NOT DO THIS
    rosdep update
    ```

2. Unzip the package if needed.Create the workspace as demostrated.
Place the contents of the hw3_ws/src folder into the ROS workspace under src folder:

    ```bash
    mkdir -p jackal_ws/src
    cd jackal_ws/src
    ```

3. Installed required packages via rosdep and git clone  and build workspace:

    ```bash
    cd ..
    git clone https://github.com/theunknowninfinite/jackal_teleop.git
    git clone https://github.com/theunknowninfinite/jackal_simulator.git
    git clone https://github.com/theunknowninfinite/jackal.git
    rosdep install -i --from-path src --rosdistro noetic -y
    sudo apt-get install ros-noetic-jackal-* ros-noetic-velodyne-*
    cd ..
    catkin_make
    ```

4. Source the package as
    ```bash
    source jackal_ws/devel/setup.bash
    ```

## Running the Teleop Launch File

To run the teleop launch file, follow these steps:

1. Launch the gazebo & teleop node:

    ```bash
    roslaunch jackal_teleop teleop.launch
    ```
The gazebo window will start loading while another terminal window will pop up that will allow you to teleop
the robot.
Pressing E will close the terminal window.
Controls:
- WASD to navigate
- E to stop and shutdown node

# Part Two
## Running the Move Base Launch File

To run the move base launch file, follow these steps:

1. Launch the move base node:

    ```bash
    roslaunch jackal_teleop move_base.launch
    ```
This will launch the gazebo window while the script to pass goals to the robot opens in a new terminal window.

This also causes rviz to open up so that the cost map , loaded map and the path that the robot has decided to take to
reach the goal point can be visualised.
The terminal window shows the goal points given and the status if it reached the goal or not. This window will automatically close once the robot has reached
the final point.
The robot now begins to navigate in the environment. The progress can be seen in rviz.
That's it!

## Running the Docker File

Ensure that you are using a X11 based desktop environment and you have enabled display output for docer containers by running

    ```bash
    xhost +local:*
    ```
1. For part one , ensure the dockerfile has been extracted from the zip and run the following command,
    ```bash
        docker build -f Dockerfile -t test_image . && clear && docker run --rm -it --net=host --ipc=host --pid=host --privileged -v /dev/shm:/dev/shm -e DISPLAY=$DISPLAY -v /dev/input:/dev/input:rw -v /tmp/.X11-unix:/tmp/.X11-unix:ro  --name=sim_dock test_image:latest /bin/bash -c "source devel/setup.bash && roslaunch jackal_teleop teleop.launch"
    ```
    A docker image will be built and a container will be launched, allowing you to teleop, the instructions for the same have been given above.

2. For part two the same dockerfile can be used but with the following command,

    ```bash
    docker build -f Dockerfile -t test_image . && clear && docker run --rm -it --net=host --ipc=host --pid=host --privileged -v /dev/shm:/dev/shm -e DISPLAY=$DISPLAY -v /dev/input:/dev/input:rw -v /tmp/.X11-unix:/tmp/.X11-unix:ro  --name=sim_dock test_image:latest
    ```
    A docker image will be built and a container will be launched, allowing you to see the output as described above.


## Note
Sometimes for part two , the robot can get confused in it's localisation and get stuck or take a long time to navigate ,
in that case, close all terminals and try runnning it again.

If the docker image fails to build, please try adding the flags --no-cache --pull to it. This will ensure a fresh image is rebuilt without using cache.

## References and Credits

1. ROS Documentation

2. Clearpath Robotics Documentation for [Jackal](https://docs.clearpathrobotics.com/docs/ros1noetic/robots/outdoor_robots/jackal/tutorials_jackal/#introduction)

3. Jackal [Github](https://github.com/jackal) repository for Jackal, Jackal Navigation and Jackal Simulator Package.
Parts of the Jackal Navigation package was used for launching Rviz for previews,creating a map of the Gazebo world and parameters required for movebase, gslam and AMCL and their respective launch files. The jackal simulator and jackal package were used to spawn the robot in a custom world for teleop and navigation.

4. https://answers.ros.org/question/30754/launching-nodes-in-a-new-terminal-with-roslaunch/  for lauching nodes in seperate terminal windows

5. https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py/ for inspiration for code for sending goals to the action server for navigation via python.

6. https://github.com/aws-robotics/aws-robomaker-small-house-world for the world and models used in Gazebo for the simulation.

7. Docker documentation and Display Forwarding [Link](https://stackoverflow.com/questions/44429394/x11-forwarding-of-a-gui-app-running-in-docker)
