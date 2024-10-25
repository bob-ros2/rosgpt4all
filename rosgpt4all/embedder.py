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
import json
import logging
import traceback
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor

# ros2 topic pub --once embed std_msgs/msg/String 'data: "{\"collection\":\"xyz\", \"documents\":[\"hey\"], \"metadatas\": [{\"name\":\"hans\"}]}"'

class EmbedderNode(Node):
    """Basic Embedder ROS Node.
    This ROS node subscribes to an /embed String topic to receive JSON data with embedding messages.
    It embeds the delivered data into the configured Vector DB.
    A Qdrant Vector DB is the default DB.
    To embed into a Chroma DB set the environment variable EMBED_CHROMADB, it's enough if it just exists.
    See ROS parameter for further configuration.
    The JSON data has to contain the following fields. (for Qdrant the ids are optional) 
    {"collection":"xyz", "documents":["hey ya","jo do"], "metadatas": [{"name":"works"}, {"name":"nut"}], "ids":['id1','id2']}
    """

    def __init__(self):
        super().__init__('embedder')
        logging.basicConfig(
            level = (logging.DEBUG
                if self.get_logger().get_effective_level() == LoggingSeverity.DEBUG \
                else logging.INFO),
            format="[%(levelname)s] [%(asctime)s.] [%(name)s]: %(message)s",
            datefmt="%s")

        self.declare_parameter(
            'model', '', 
            ParameterDescriptor(description=
            'To be used embedding model.'))
        self.declare_parameter(
            'host', 'localhost', 
            ParameterDescriptor(description=
            'Vector DB host name or ip address.'))
        self.declare_parameter(
            'port', (8000
                if os.getenv('EMBED_CHROMADB',None) != None 
                else 6333), 
            ParameterDescriptor(description=
            'Vector DB port.'))
        self.declare_parameter(
            'path', '', 
            ParameterDescriptor(description=
            'Vector DB local path if using persistent storage.'))
        self.declare_parameter(
            'location', '', 
            ParameterDescriptor(description=
            'Vector DB location. Can be empty or url'))

        self.model = self.get_parameter(
            'model').get_parameter_value().string_value or None
        self.host = self.get_parameter(
            'host').get_parameter_value().string_value
        self.port = self.get_parameter(
            'port').get_parameter_value().integer_value  
        self.path = self.get_parameter(
            'path').get_parameter_value().string_value
        self.location = self.get_parameter(
            'location').get_parameter_value().string_value

        if os.getenv('EMBED_CHROMADB', None) != None:
            self.embed = self.init_chroma()
        else:
            self.embed = self.init_qdrant()

        # subscribe to embed topic
        self.sub = self.create_subscription(
            String, 'embed', self.embed_callback, 1000)
        # subscribe to embed_raw topic
        self.sub_raw = self.create_subscription(
            String, 'embed_raw', self.embed_raw_callback, 1000)

        self.get_logger().info('Init done, waiting for embed messages')
        
    def init_qdrant(self):
        """Init Qdrant embedder
        """
        from qdrant_embedder import QdrantEmbedder
        self.get_logger().info('using Qdrant DB')
        if self.location:
            return QdrantEmbedder(
                self.model, prefer_grpc=True, location=self.location)
        elif self.path:
            return QdrantEmbedder(
                self.model, prefer_grpc=True, path=self.path)
        elif self.host and self.port:
            return QdrantEmbedder(
                self.model, prefer_grpc=True, host=self.host, port=self.port)
        elif self.host:
            return QdrantEmbedder(
                self.model, prefer_grpc=True, host=self.host)
        else:
            return QdrantEmbedder(
                self.model, prefer_grpc=True)

    def init_chroma(self):
        """Init Chroma embedder
        """
        from chroma_embedder import ChromaEmbedder
        self.get_logger().info('using Chroma DB')
        if self.path:
            return ChromaEmbedder(
                self.model, path=self.path)
        elif self.host and self.port:
            return ChromaEmbedder(
                self.model, host=self.host, port=str(self.port))
        elif self.host:
            return QdrantChromaEmbedderEmbedder(
                self.model, host=self.host)
        else:
            return QdrantEmbedder(
                self.model)

    def embed_callback(self, msg: String) -> None:
        """Embeds the incoming JSON topic message into the Vector DB.
        Prints the incoming message also to the debug log.
        If it fails an error log entry will be written.
        """
        try:
            self.get_logger().info("embedding data")
            self.embed(msg.data)
            self.get_logger().debug(
                f'embed_callback: {msg.data}')
        except Exception as e: 
            self.get_logger().error(
                f'embed_callback: {traceback.format_exc()}')

    def embed_raw_callback(self, msg: String) -> None:
        """Embeds raw data of any type into default collection 'embed_raw'.
        To use another collection set the env var EMBED_COLLECTION accordingly.
        If the collection comes with the input data that will be used instead.
        """
        embed_msg = {}
        try:
            data = json.loads(msg.data)
            if  'collection' in data \
                and 'documents' in data \
                and 'metadatas' in data:
                self.embed_callback(msg)
            elif isinstance(data, dict):
                embed_msg['collection'] = (data['collection'] 
                    if 'collection' in data else os.getenv(
                        'EMBED_COLLECTION', 'embed_raw'))
                embed_msg['documents'] = (data['documents'] 
                    if 'documents' in data else [(data['data']
                        if 'data' in data else json.dumps(data))])
                embed_msg['metadatas'] = [data]
                if 'ids' in data:
                    embed_msg['ids'] = data['ids'] 
                self.embed_callback(
                    String(data=json.dumps(embed_msg)))
            else: raise Exception('none')
        except:
            embed_msg['collection'] = os.getenv(
                'EMBED_COLLECTION', 'embed_raw')
            embed_msg['documents'] = [msg.data]
            embed_msg['metadatas'] = [{'data': msg.data}]
            self.embed_callback(json.dumps(embed_msg))


def main():
    rclpy.init(args=None)
    n = EmbedderNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
