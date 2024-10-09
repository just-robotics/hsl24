ARG base_img

FROM ${base_img}

RUN apt update -y && apt install -y ros-humble-nav2-msgs \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox 

RUN apt pip install -y && python3 -m pip install opencv-contrib-python 

RUN echo "alias s='source install/setup.bash'" >> ~/.bashrc
RUN echo "alias b='colcon build --symlink-install'" >> ~/.bashrc
RUN echo "alias c='clear'" >> ~/.bashrc

### ---------------------------------------------------------------------- #
