FROM althack/ros2:foxy-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt update \
   && apt -y install --no-install-recommends git-flow ros-foxy-webots-ros2 \
   #
   # Clean up
   && apt autoremove -y \
   && apt clean -y \
   && rm -rf /var/lib/apt/lists/* \
   && pip3 install rpi.gpio \
   && pip3 install mock.gpio

ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc