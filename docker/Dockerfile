# rich base image
FROM osrf/ros:noetic-desktop-full

# install catkin tools first
RUN apt update && apt install -y \
    python3-catkin-tools \ 
    bash-completion \ 
    htop

# copy project to container temporarily for installing dependencies
COPY src /home/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    rosdep update --rosdistro=noetic && \
    rosdep install -y --from-paths /home/src --ignore-src --rosdistro noetic"

# install dependencies
COPY Makefile /home/Makefile
COPY shell /home/shell
RUN cd /home && make install_deps

# remove /home/src and /home/Makefile after installing dependencies
RUN rm -rf /home/src
RUN rm /home/Makefile
RUN rm -rf /home/shell

# set entrypoint including process of sourcing ros environment
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
