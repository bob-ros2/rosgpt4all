ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

USER 0
ARG USER=ros
ARG UID=1000
ARG GID=1000
ARG WORKSPACE=colcon_ws

# prepare user & group
RUN groupadd -g $GID $USER \
    && useradd -ms /bin/bash -u $UID -g $USER $USER \
    && usermod -a -G audio $USER

RUN apt-get update && apt-get install --no-install-recommends -y \
    git python3-pip 

RUN echo "$USER ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USER

COPY ./entrypoint.sh /home/$USER/entrypoint.sh
RUN chown $USER:$USER /home/$USER/entrypoint.sh \
  && chmod +x /home/$USER/entrypoint.sh

USER $USER
ENV USER=$USER
ENV WORKSPACE=$WORKSPACE
ENV XDG_RUNTIME_DIR=/tmp/runtime-ros
WORKDIR /home/$USER

# setup workspace
RUN mkdir -p /home/$USER/$WORKSPACE/src \
  && mkdir -p /home/$USER/.local/share/nomic.ai/GPT4All \
  && cd /home/$USER/$WORKSPACE/src \
  && git clone https://gitlab.com/bob-ros2/rosgpt4all.git \
  #&& pip3 install --no-cache-dir -r ~/$WORKSPACE/src/rosgpt4all/requirements.txt 
  && pip3 install --no-cache-dir gpt4all==2.6.0

# build package(s)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
  && cd /home/$USER/$WORKSPACE && colcon build 

# set .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USER/.bashrc \
  && echo "source /home/$USER/$WORKSPACE/install/setup.bash" >> /home/$USER/.bashrc

ENTRYPOINT ["/home/ros/entrypoint.sh"]
