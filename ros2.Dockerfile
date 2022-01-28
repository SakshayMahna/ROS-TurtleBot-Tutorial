FROM tiryoh/ros2-desktop-vnc:dashing

# Setup NON INTERACTIVE ENVIRONMENT
ARG DEBIAN_FRONTEND=noninteractive

# In order to be able to source
SHELL ["/bin/bash", "-c"]

# Setup Tutorial
RUN mkdir -p ~/Desktop/turtlebot3_ws/src
RUN cd ~/Desktop/turtlebot3_ws/src && \
    git clone -b dashing-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone -b dashing-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

ENV MAKEFLAGS="-j1"
RUN source /opt/ros/dashing/setup.bash && \
    cd ~/Desktop/turtlebot3_ws && colcon build
RUN echo 'source ~/Desktop/turtlebot3_ws/install/setup.bash' >> ~/.bashrc

# Map Tutorial
RUN sudo apt-get update
RUN sudo apt-get install -y ros-dashing-cartographer-ros
RUN sudo apt-get install -y ros-dashing-nav2-map-server

# Navigation Tutorial
RUN sudo apt-get install -y ros-dashing-navigation2 \
                            ros-dashing-nav2-bringup

# Install Text Editor
RUN sudo apt-get install -y gedit

EXPOSE 6080