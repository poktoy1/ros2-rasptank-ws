FROM althack/ros2:foxy-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && apt upgrade -y && rosdep update \
   && apt -y install --no-install-recommends nvidia-driver-470 software-properties-common \
   git-flow ros-foxy-webots-ros2 \
   && wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add - \
   && apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/' \
   && apt update && apt upgrade \
   && apt install -y webots python3-rosdep2 python3-vcstool \
   #
   # Clean up
   && apt autoremove -y \
   && apt clean -y \
   && rm -rf /var/lib/apt/lists/* \
   && pip3 install rpi.gpio \
   && pip3 install mock.gpio \
   && pip3 install opencv-python
   

ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc