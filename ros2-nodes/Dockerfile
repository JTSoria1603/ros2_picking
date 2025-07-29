FROM ros:humble

ENV DEBIAN_FRONTEND=nointeractive


RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    build-essential \
    git \
    curl \
    nano \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/ros2_picking

RUN apt update && apt install -y tmux

COPY ./ros2_picking ./

RUN . /opt/ros/humble/setup.sh && \
    colcon build

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

SHELL [ "/bin/bash", "-c" ]

CMD bash -c "source /opt/ros/humble/setup.bash && \
             source /home/ros2_picking/install/setup.bash && \
             ros2 launch cell_launch cell.launch.py"
