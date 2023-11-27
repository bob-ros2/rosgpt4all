# ROS Package RosGPT4all

This package contains [ROS](https://ros.org) Nodes related to the open source 
project [GPT4ALL](https://github.com/nomic-ai/gpt4all).

It integrates a GPT chat into the ROS framework. It enables to communicate via 
topics or UNIX socket with a running GPT4ALL chat session.\
Additionally a custom configurable terminal window ROS node is available to 
enter chat messages and display the GPT response similar to the existing 
GPT4ALL GUI but with reduced features.\
This package is not another GPT4all variant. It just make use of it and can be seen as a wrapper for ROS.

Related links:
- https://github.com/nomic-ai/gpt4all
- https://ros.org

## Index
<!-- https://ecotrust-canada.github.io/markdown-toc/ -->
- [ROS Package RosGPT4all](#ros-package-rosgpt4all)
  * [Index](#index)
  * [Dependencies](#dependencies)
  * [Setup package](#setup-package)
- [ROS Node GPT](#ros-node-gpt)
  * [Usage](#usage)
  * [Node Parameter](#node-parameter)
  * [Subscribed Topics](#subscribed-topics)
  * [Published Topics](#published-topics)
  * [Socket stream](#socket-stream)
- [ROS Node TERMINAL](#ros-node-terminal)
  * [Dependencies](#dependencies-1)
  * [Usage](#usage-1)
  * [Node Parameter](#node-parameter-1)
  * [Subscribed Topics](#subscribed-topics-1)
  * [Published Topics](#published-topics-1)
- [Contributing](#contributing)

## Dependencies
```bash
# install gpt4all
pip install gpt4all

# if using embedding
pip install langchain
# install chroma
sudo apt install chroma
# install chromadb
pip install chromadb
pip install unstructured
```

## Setup package
```bash
# run in your ros2_ws/src folder
git clone https://gitlab.com/bob-ros2/rosgpt4all.git
cd ..
colcon build
. install/setup.bash
```

# ROS Node GPT

## Usage
```bash
# run as server ROS node
ros2 run rosgpt4all gpt.py

# run with redirect produced output
ros2 run rosgpt4all gpt.py --ros-args -r gpt_out:=/speak

# send message to gpt4all via command line
ros2 topic pub --once /gpt_in std_msgs/msg/String "{data: 'Hello'}"
```
## Node Parameter

> ~allow_download\
  Type: string\
  Allow API to download models from gpt4all.io.\
  Default: true

> ~device\
  Type: string\
  The processing unit on which the GPT4All model will run. It can be set to:\
  "cpu": Model will run on the central processing unit.\
  "gpu": Model will run on the best available graphics processing unit, irrespective of its vendor.\
  "amd", "nvidia", "intel": Model will run on the best available GPU from the specified vendor.\
  Alternatively, a specific GPU name can also be provided, and the model will run on the GPU that matches the name if it's available.\
  Default: cpu

> ~eof_indicator\
  Type: string\
  End of file indicator. It will be send when the generator task was 
  done. (only gpt_generator and gpt_sentence topics). Leave blank if no EOF 
  should be send.\
  Default: EOF

> ~max_tokens\
  Type: integer\
  The maximum number of tokens to generate.\
  Default: 200

> ~model_name\
  Type: string\
  GPT4ALL model to be used. On the node very first start it will be dowloaded 
  if it does not exist in ~model_path. Auto download depends on ~allow_download.\
  Default: orca-mini-3b.ggmlv3.q4_0.bin

> ~model_path\
  Type: string\
  GPT4ALL model path to be used.\
  Default: ~/.local/share/nomic.ai/GPT4All

> ~n_batch\
  Type: integer\
  Number of prompt tokens processed in parallel. Larger values decrease latency 
  but increase resource requirements.\
  Default: 8

> ~prompt\
  Type: string\
  Promp format string to be used.\
  Default: > {0}\n\n

> ~repeat_last_n\
  Type: integer\
  How far in the models generation history to apply the repeat penalty.\
  Default: 64

> ~repeat_penalty\
  Type: double\
  Penalize the model for repetition. Higher values result in less repetition.\
  Default: 1.18

> ~socket_path\
  Type: string\
  Unix socket where to write the output.\
  Default: /tmp/gpt.sock

> ~temp\
  Type: double\
  The model temperature. Larger values increase creativity but decrease 
  factuality.\
  Default: 0.7

> ~top_k\
  Type: integer\
  Randomly sample from the top_k most likely tokens at each generation step. 
  Set this to 1 for greedy decoding.\
  Default: 40

> ~top_p\
  Type: double\
  Randomly sample at each generation step from the top most likely tokens whose 
  probabilities add up to top_p.\
  Default: 0.4

## Subscribed Topics

> ~gpt_in (std_msgs/String)\
GPT input.

## Published Topics

> ~gpt_out (std_msgs/String)\
GPT ouput. The whole message will be published when the generator has finished.

> ~gpt_generator (std_msgs/String)\
GPT generator ouput. Each single token is published to the topic.

> ~gpt_sentence (std_msgs/String)\
GPT ouput aggregated as sentence or sub sentence. A message with content EOF indicates the end.
of the generator output.

## Socket stream
The GPT node also creates a socket where the generator output will be streamed 
in realtime. Multiple clients can connect.
```bash
# the generator output can be received as followed
netcat -v -U /tmp/gpt.sock 
```

# ROS Node TERMINAL
With this Ros Node the generator output of GPT4ALL node can be received and displayed in realtime. This works as well with any other stdout stream. Also a topic input subscriber is available. An input field can optionally be turned on which can be used to publish messages to another topic.

## Dependencies
The required QT5 libraries should already exist if ROS is installed. If 
missing use below installation to get them.
```bash
sudo apt-get install python3-pyqt5
```

## Usage
```bash
# connect terminal with GPT node socket stream, use one of the existing QT5 arguments
netcat -v -U /tmp/gpt.sock | python3 terminal.py -qwindowgeometry 640x480+600+300

# run frameless terminal window using ROS parameter
netcat -v -U /tmp/gpt.sock | ros2 run rosgpt4all terminal.py --ros-args -p frameless:=true -p geometry:=[300,300,600,480]

# show window on another display
netcat -U /tmp/gpt.sock | ros2 run rosgpt4all terminal.py --ros-args -p display:=1 -p geometry:=[300,300,600,480]
```

## Node Parameter

> ~display\
  Type: integer\
  Display number where to show the window. -1 = default display.\
  Default: -1

> ~fontname\
  Type: string\
  Window fontname.\
  Default: courier

> ~fontsize\
  Type: integer\
  Window fontsize.\
  Default: 12

> ~frameless\
  Type: boolean\
  Switch off window caption.\
  Default: false

> ~geometry\
  Type: integer array\
  Window geometry. [x, y, with, height]

> ~margin\
  Type: integer array\
  Window inner margin. [left, top, right, bottom]\
  Default: [10,0,0,0]

> ~input\
  Type: boolean\
  Enables or disables the text input field.\
  Default: true

> ~opacity\
  Type: double\
  Window opacity. 1.0 = fully visible.\
  Default: 1.0

> ~stylesheet\
  Type: string\
  Stylesheet qss of PlainText area.\
  Default: background-color: #000000; color: #f0f0f0;

> ~title\
  Type: string\
  Title of window.\
  Default: GPT4ALL Terminal

> ~line_count\
  Type: string\
  Maximum line count in the text area. 0 = unlimited.\
  If the number exceeds the lines are removed from the top.
  Default: 0

## Subscribed Topics

> ~input (std_msgs/String)\
Read input data from topic in addition to be able to read data from stdin.

## Published Topics

> ~gpt_in (std_msgs/String)\
Publish to GPT4ALL input.

# Contributing
Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.\
Please make sure to update tests as appropriate.
