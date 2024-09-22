FROM pengo-base:demo


COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]

WORKDIR /root/qualcomm_ws

# RUN apt-get update

# WORKDIR /pengo-setup/src
# RUN git clone https://github.com/kfir97/pengo-setup.git
# WORKDIR /pengo-setup/src/pengo-setup
# RUN git pull
# WORKDIR /pengo-setup
# RUN apt-get update
# RUN . /opt/ros/melodic/setup.sh && catkin_make



# ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/melodic/setup.bash && source /pengo-setup/devel/setup.bash && exec $0 $@"]

# CMD ["roslaunch", "robot_setup", "robot_setup.launch"]
