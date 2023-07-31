# docker pull osrf/ros:foxy-desktop
# docker pull osrf/ros:noetic-desktop


# docker images to view all the installed images**
# docker run -it osrf/ros:noetic-desktop```  To change the cmd format from Windows to a Linux based system**
# lsb_release -a To check current version of ubuntu** 

# docker built -t <image name >.
FROM osrf/ros:foxy-desktop
RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip
RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src/

RUN git clone https://github.com/krinkin/cyprus-int-2023 && \
    cd ~/catkin_ws
RUN echo "CY2023!"
