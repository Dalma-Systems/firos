FROM ros:melodic-ros-base

RUN mkdir catkin_ws \
    && mkdir catkin_ws/src \
    && mkdir catkin_ws/src/firos \
    && mkdir catkin_ws/devel \
    && mkdir catkin_ws/build

COPY . /catkin_ws/src/firos
WORKDIR /catkin_ws

# remove descriptions/whitelist and robots configuration, since those need to be added by the user
RUN rm -f /catkin_ws/src/firos/config/robots.json && \
    rm -f /catkin_ws/src/firos/config/robotdescriptions.json && \
    rm -f /catkin_ws/src/firos/config/whitelist.json

# install ros packages
RUN apt-get update --allow-insecure-repositories && apt-get install python-pip ros-melodic-tf  -y --allow-unauthenticated
#RUN apt-get update && apt-get install -y ros-melodic-ros-base python python-pip
RUN pip install -r /catkin_ws/src/firos/requirements.txt
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'


EXPOSE 10100
CMD /bin/bash -c '. /opt/ros/melodic/setup.bash; . /catkin_ws/devel/setup.bash; rosrun firos core.py'





