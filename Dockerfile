FROM ros:humble


RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-hardware-interface \
    ros-humble-xacro \
    libi2c-dev \
    i2c-tools \
    unzip \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y build-essential wget && \
    wget https://github.com/joan2937/pigpio/archive/master.zip && \
    unzip master.zip && \
    cd pigpio-master && \
    make && \
    make install && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /rusmed_ws
COPY . /rusmed_ws/

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/humble/setup.bash && colcon build

COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
