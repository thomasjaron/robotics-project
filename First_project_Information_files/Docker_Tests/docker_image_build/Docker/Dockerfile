# syntax=docker/dockerfile:1
FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt -y update 
RUN apt -y upgrade
RUN apt -y install tmux nano vim curl gnupg
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt -y update 
RUN apt install -y ros-noetic-desktop-full
RUN apt install -y python3-rosdep
RUN apt install -y vim nano gedit 
#RUN apt install -y ros-noetic-rqt* ros-noetic-jsk* ros-noetic-visualization-tutorials ros-noetic-move-base
#RUN apt -y update && apt install -y ros-noetic-plotjuggler-ros


ARG UNAME=evaluator
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME
USER $UNAME
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /home/evaluator/robotics/devel/setup.bash" >> ~/.bashrc
COPY eval.py /home/evaluator/eval.py
COPY ros-entrypoint.sh ros-entrypoint.sh

ADD ros-entrypoint.sh /ros-entrypoint.sh
ENTRYPOINT [ "/ros-entrypoint.sh" ]
CMD python3 /home/evaluator/eval.py
