#!/usr/bin/env python3
#
# Copyright 2023 BobRos
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import os
import glob
import time
import socket
from threading import Thread
# https://github.com/nomic-ai/gpt4all
# https://docs.gpt4all.io/gpt4all_python.html
from gpt4all import GPT4All
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy

class GPTNode(Node):
    """
    GPT4All ROS Node.
    """  

    def __init__(self):

        super().__init__('gpt')

        self.running = True
        self.socket = None
        self.socket_clients = []
        self.queue = []
        self.gpt_abort = False

        # model related parameter

        self.declare_parameter('model_name', 
            #'gpt4all-13b-snoozy-q4_0', 
            'orca-mini-3b-gguf2-q4_0',
            ParameterDescriptor(description=
            'GPT4ALL model to be used.'))
        self.model_name = self.get_parameter(
            'model_name').get_parameter_value().string_value 

        self.declare_parameter('model_path', 
            os.path.expanduser('~') + \
                '/.local/share/nomic.ai/GPT4All', 
            ParameterDescriptor(description=
            'GPT4ALL model path to be used.'))
        self.model_path = self.get_parameter(
            'model_path').get_parameter_value().string_value

        self.declare_parameter('allow_download', True, 
            ParameterDescriptor(description=
            'Allow dowloading model if it doesnt exist.'))
        self.allow_download = self.get_parameter(
            'allow_download').get_parameter_value().bool_value
        
        # check model first before going on

        self.get_logger().info(f"using {self.model_name}")
        path = os.path.join(self.model_path, self.model_name)
        if not glob.glob(f"{path}*"):
            if self.allow_download:
                self.get_logger().info(
                    'the model does not yet exists, '
                    'downloading may take a while ...')
            else: 
                self.get_logger().error(
                    'the model does not exists '
                    'and allow_download is set to false. '
                    'Please place the model in the model dir '
                    'or set ROS param allow_download to true')
                sys.exit(1)

        # other ROS parameter

        self.declare_parameter(
            'socket_path', '/tmp/gpt.sock', 
            ParameterDescriptor(description=
            'Unix socket where to write the output.'))
        self.socket_path = self.get_parameter(
            'socket_path').get_parameter_value().string_value

        self.declare_parameter('device', 'cpu', 
            ParameterDescriptor(description=
            'The processing unit on which the GPT4All model will '
            'run. It can be set to:\n'
            ' cpu: Model will run on the central processing unit.\n'
            ' gpu: Model will run on the best available graphics '
            'processing unit, irrespective of its vendor.\n'
            ' amd, nvidia, intel: Model will run on the best '
            'available GPU from the specified vendor.'))
        self.device = self.get_parameter(
            'device').get_parameter_value().string_value

        self.declare_parameter('system_prompt', 
            "You are a helpful assistant with the name BobRos using model " + \
                self.get_parameter(
                    'model_name').get_parameter_value().string_value,
            ParameterDescriptor(description=
            'System promp to be used.'))
        self.system_prompt = self.get_parameter(
            'system_prompt').get_parameter_value().string_value

        self.declare_parameter('prompt', '> {0}\n\n', 
            ParameterDescriptor(description=
            'Promp format string to be used. E.g.: > {0}\\n\\n'))
        self.prompt = self.get_parameter(
            'prompt').get_parameter_value().string_value

        self.declare_parameter('eof_indicator', '', 
            ParameterDescriptor(description=
            'End of file indicator. It will be send when the '
            'generator task was done. (only gpt_generator and gpt_sentence '
            'topics). Leave blank if no EOF should be send.'))
        self.eof_indicator = self.get_parameter(
            'eof_indicator').get_parameter_value().string_value

        # gpt4all parameter generator

        self.declare_parameter('max_tokens', 200, 
            ParameterDescriptor(description=
            'The maximum number of tokens to generate.'))

        self.declare_parameter('temp', 0.7, 
            ParameterDescriptor(description=
            'The model temperature. Larger values increase creativity but '
            'decrease factuality.'))

        self.declare_parameter('top_k', 40, 
            ParameterDescriptor(description=
            'Randomly sample from the top_k most likely tokens at each '
            'generation step. Set this to 1 for greedy decoding.'))

        self.declare_parameter('top_p', 0.4, 
            ParameterDescriptor(description=
            'Randomly sample at each generation step from the top most likely '
            'tokens whose probabilities add up to top_p.'))

        self.declare_parameter('repeat_penalty', 1.18, 
            ParameterDescriptor(description=
            'Penalize the model for repetition. Higher values result in '
            'less repetition.'))

        self.declare_parameter('repeat_last_n', 64, 
            ParameterDescriptor(description=
            'How far in the models generation history to apply the '
            'repeat penalty.'))

        self.declare_parameter('n_threads', 0, 
            ParameterDescriptor(description=
            'Number of CPU threads used by GPT4All. Default is None, '
            'then the number of threads are determined automatically.'))

        self.declare_parameter('verbose', False, 
            ParameterDescriptor(description=
            'If True, print debug messages.'))
        
        self.declare_parameter('ngl', 0, 
            ParameterDescriptor(description=
            'Number of GPU layers to use (Vulkan).'))

        self.declare_parameter('n_batch', 8, 
            ParameterDescriptor(description=
            'Number of prompt tokens processed in parallel. Larger values '
            'decrease latency but increase resource requirements.'))

        self.max_tokens = self.get_parameter(
            'max_tokens').get_parameter_value().integer_value
        self.temp = self.get_parameter(
            'temp').get_parameter_value().double_value
        self.top_k = self.get_parameter(
            'top_k').get_parameter_value().integer_value
        self.top_p = self.get_parameter(
            'top_p').get_parameter_value().double_value
        self.repeat_penalty = self.get_parameter(
            'repeat_penalty').get_parameter_value().double_value
        self.repeat_last_n = self.get_parameter(
            'repeat_last_n').get_parameter_value().integer_value
        self.n_batch = self.get_parameter(
            'n_batch').get_parameter_value().integer_value
        self.n_threads = (self.get_parameter(
            'n_threads').get_parameter_value().integer_value)
        self.n_threads = self.n_threads if self.n_threads > 0 else None
        self.ngl = self.get_parameter(
            'ngl').get_parameter_value().integer_value
        self.verbose = self.get_parameter(
            'verbose').get_parameter_value().bool_value

        # end gpt4all parameter generator
 
        self.add_on_set_parameters_callback(
            self.parameter_callback)

        self.sub = self.create_subscription(
            String, 'gpt_in', 
            self.topic_callback, 1000)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
            depth=1000)

        self.pub_generator = self.create_publisher(
            String, 'gpt_generator', qos_profile)

        self.pub = self.create_publisher(
            String, 'gpt_out', 1000)

        self.pub_sentence = self.create_publisher(
            String, 'gpt_sentence', 1000)

        self.sentence = ''

        self.model = GPT4All(
            model_name     = self.model_name,
            model_path     = self.model_path,
            allow_download = self.allow_download,
            device         = self.device,
            n_threads      = self.n_threads,
            ngl            = self.ngl,
            verbose        = self.verbose
        )

        self.session_thread = Thread(target=self.session)
        self.sock_thread = Thread(target=self.socket_server)

        self.session_thread.start()
        self.sock_thread.start()
        self.get_logger().info('init done')


    def gpt_callback(self, token_id: int, response: str):
        """
        Callback to handle self.gpt_abort signal.
        """

        if self.gpt_abort:
            return False
        return True


    def shutdown(self):
        """
        Cleanup running threads.
        """

        self.running = False
        self.session_thread.join()
        self.sock_thread.join()


    def parameter_callback(self, params):
        """
        ROS dynamic reconfigure handler for RQT Gui
        """

        for param in params:
            if   param.name == "max_tokens": self.max_tokens = param.value
            elif param.name == "temp": self.temp = param.value
            elif param.name == "top_k": self.top_k = param.value
            elif param.name == "top_p": self.top_p = param.value
            elif param.name == "repeat_penalty": self.repeat_penalty = param.value
            elif param.name == "repeat_last_n": self.repeat_last_n = param.value
            elif param.name == "n_batch": self.n_batch = param.value
            elif param.name == "eof_indicator": self.eof_indicator = param.value

        return SetParametersResult(successful=True)


    def topic_callback(self, msg):
        """
        Pass incoming message to GPT4ALL.
        """

        if msg.data in ['stop','shutup','shut up']:
            self.gpt_abort = True
        else:
            self.queue.append(msg.data)


    def prompt_cleanup(self, prompt):
        """
        Removes context until the end from prompt.
        """

        return prompt.split("### Context:", 1)[0]
        

    def session(self):
        """
        Process incoming queue messages by GPT4ALL.
        """

        with self.model.chat_session(self.system_prompt):
            while self.running:
                time.sleep(0.1)
                if len(self.queue):

                    output = ''
                    self.gpt_abort = False
                    prompt = self.queue.pop(0)

                    if prompt == 'clear':
                        self.model.current_chat_session.clear()
                        continue

                    prompt = self.prompt.format(prompt)
                    query = self.prompt_cleanup(prompt)

                    self.token_handler(query)
                    self.print(query)

                    for token in self.model.generate(
                        prompt,
                        max_tokens = self.max_tokens,
                        temp = self.temp,
                        top_k = self.top_k,
                        top_p = self.top_p,
                        repeat_penalty = self.repeat_penalty,
                        repeat_last_n = self.repeat_last_n,
                        n_batch = self.n_batch,
                        streaming = True,
                        callback  = self.gpt_callback):
                        # handle returned single token
                        self.print(token)
                        self.token_handler(token)
                        output += token

                    # finalize
                    self.print("\n\n")
                    self.token_handler("\n\n")
                    self.token_handler('EOF')
                    self.publish(output, self.pub)


    def token_handler(self, token):
        """
        Publishes to the generator and sentence publishers.
        """

        SENTENCE_DELIMETER = ".:,;!?-*\n\t"

        if token == 'EOF':
            if self.eof_indicator:
                self.publish(self.sentence+self.eof_indicator, self.pub_sentence)
                self.publish(self.eof_indicator, self.pub_generator)
            else: 
                self.publish(self.sentence, self.pub_sentence)
            self.sentence = ''
            return
        else:
            self.publish(token, self.pub_generator)

        for c in token:
            self.sentence += c
            if c in SENTENCE_DELIMETER:
                self.publish(self.sentence, self.pub_sentence)
                self.sentence = ''


    def publish(self, s, pub):
        """
        Publish GPT4ALL response.
        """

        msg = String()
        msg.data = s
        pub.publish(msg)
        self.get_logger().debug('Published: %s' % s)


    def print(self, s):
        """
        Print to stdout and socket.
        """

        print(s, end="", flush=True)
        if self.socket:
            try: self.socket_send(s.encode())
            except: pass


    # socket related

    def socket_server(self):
        """
        Serves GPT output to a AF_UNIX socket.
        """

        while self.running:
            try: 
                if not self.socket:
                    self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM, 0)
                    if os.path.exists(self.socket_path):
                        os.remove(self.socket_path)
                    self.socket.bind(self.socket_path)
                self.socket.listen()
                self.socket.setblocking(0)
                self.socket.settimeout(0.2)
                self.socket_clients.append(self.socket.accept())
            except: continue
        if self.socket: self.socket.close()


    def socket_send(self, bytes):
        """
        Send bytes to all connected clients.
        Also maintains the self.socket_clients list.
        """

        def send(c, bytes):
            try: return c.send(bytes)
            except: return None

        self.socket_clients = [c for c in self.socket_clients if send(c[0], bytes)]


def main():

    try:
        rclpy.init(args=None)
        n = GPTNode()
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass

    n.shutdown()

if __name__ == '__main__':
    main()
