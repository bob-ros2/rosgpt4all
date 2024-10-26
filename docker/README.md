# ROS GPT4ALL Dockerfile

this Dockerfile integrates GPT4ALL into a ROS (Robot Operating System) environment

Related links:
- https://github.com/nomic-ai/gpt4all
- https://docs.gpt4all.io/
- https://ros.org

```bash
# build the docker image in the docker folder
$ sudo docker build --no-cache -t gpt4all-ros:latest-humble .

# start rosgpt4all directly with the container
# starts additionaly a terminal to interact with the gpt and the ROS rqt GUI
# to show the windows a running X Server is needed
# maps a directory for the models
# if no existing model is configured a small default model will be dowloaded
sudo docker run -it --net=host \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=100 \
    -v ~/.local/share/nomic.ai/GPT4All:/home/ros/.local/share/nomic.ai/GPT4All \
    gpt4all-ros:latest-humble ros2 launch rosgpt4all gpt.launch.py terminal:=true

# run container as ROS dev environment
$ sudo docker run -it --net=host \
    -e DISPLAY=$DISPLAY \
    -v ~/.local/share/nomic.ai/GPT4All:/home/ros/.local/share/nomic.ai/GPT4All \
    gpt4all-ros:latest-humble bash

# in the container run gpt4all via launch file
$ ros2 launch rosgpt4all gpt.launch.py terminal:=true

# set another namespace
ros2 launch rosgpt4all gpt.launch.py terminal:=true ns:=/bob/gpt
```
## Pre Build Docker Image
There is also a pre build image available. Replace the image name with this one in order to use it.

> ghcr.io/bob-ros2/bob-whisper-ros:latest-humble

Package path
- https://ghcr.io/bob-ros2/gpt4all-ros
