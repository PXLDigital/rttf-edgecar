FROM enicky/cardrivers:latest-arm32v7

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update
RUN apt-get install -y ros-kinetic-rosbridge-server
RUN apt-get install -y ros-kinetic-tf2-web-republisher