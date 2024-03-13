# Use the ROS Noetic image as the base
FROM osrf/ros:noetic-desktop-full



RUN apt-get update && apt-get install -y \
    curl \
    unzip \
    python3-pip \
    git
    # python3-pip \
    #  ros-noetic-jackal-* \
    # ros-noetic-velodyne-* \
    # xterm\
    # && apt-get clean && \
    # rm -rf /var/lib/apt/lists/*

RUN pip3 install gdown

RUN mkdir -p jackal_ws/src
WORKDIR /jackal_ws/src

RUN git clone https://github.com/theunknowninfinite/jackal_teleop.git

RUN git clone https://github.com/theunknowninfinite/jackal_simulator.git

RUN git clone https://github.com/theunknowninfinite/jackal.git

WORKDIR /jackal_ws


RUN apt-get update && apt-get install -y \
    python3-rosdep

RUN rosdep update && \
    rosdep install --from-paths /jackal_ws/src --ignore-src -r -y


RUN apt-get update && apt-get install -y \
    ros-noetic-jackal-* \
    ros-noetic-velodyne-*

RUN chmod -R 755 src

RUN apt-get install xterm -y

WORKDIR /jackal_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

CMD ["/bin/bash", "-c", "source devel/setup.bash && roslaunch jackal_teleop nav.launch"]
