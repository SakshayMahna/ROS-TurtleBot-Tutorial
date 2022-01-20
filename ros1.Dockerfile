FROM tiryoh/ros-desktop-vnc:noetic

# Setup NON INTERACTIVE ENVIRONMENT
ARG DEBIAN_FRONTEND=noninteractive

# Setup Tutorial
RUN mkdir -p ~/Desktop/catkin_ws/src
RUN cd ~/Desktop/catkin_ws/src && \
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

RUN . /opt/ros/noetic/setup.sh && \
    cd ~/Desktop/catkin_ws && catkin_make
RUN echo 'source ~/Desktop/catkin_ws/devel/setup.bash' >> ~/.bashrc

# Map Tutorial
RUN sudo apt-get update --fix-missing
RUN sudo apt-get install -y ros-noetic-slam-gmapping
RUN sudo apt-get update
RUN sudo apt-get install -y ros-noetic-map-server

# Navigation Tutorial
RUN sudo apt-get install -y ros-noetic-amcl \
                            ros-noetic-move-base \
                            ros-noetic-dwa-local-planner

# Install Text Editor
RUN sudo apt-get install -y gedit