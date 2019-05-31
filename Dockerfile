FROM duckietown/rpi-duckiebot-base:master19

RUN [ "cross-build-start" ]

RUN apt-get update

RUN pip install --upgrade pip

COPY requirements.txt .

RUN python -m pip install -r requirements.txt

#Copy the package

COPY apriltags3_ros /home/software/catkin_ws/src/20-indefinite-navigation/apriltags3_ros
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash; cd /home/software/catkin_ws/; catkin_make"
RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash"

RUN [ "cross-build-end" ]
