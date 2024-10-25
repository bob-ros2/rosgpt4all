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

import sys
import select
import json
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

class VectorNode(Node):
    """
    Basic vector node
    """
    def __init__(self):
        super().__init__('vector')

        self.declare_parameter('collection_name', 'memo_embedder')
        self.declare_parameter('query', '')
        self.declare_parameter('query_model', 'nomic-ai/nomic-embed-text-v1')
        self.declare_parameter('query_url', 'http://192.168.1.134:6333')
        self.declare_parameter('query_limit', 3)
        self.declare_parameter('timer_exit', 2)

        self.collection_name = self.get_parameter(
            'collection_name').get_parameter_value().string_value
        self.query = self.get_parameter(
            'query').get_parameter_value().string_value
        self.query_model = self.get_parameter(
            'query_model').get_parameter_value().string_value
        self.query_url = self.get_parameter(
            'query_url').get_parameter_value().string_value
        self.query_limit = self.get_parameter(
            'query_limit').get_parameter_value().integer_value

        self.client = None
        self.timer = None
        self.data = None
        self.done = False

        self.sub = self.create_subscription(
            String, "query_in", self.input_callback, 10)
        self.pub = self.create_publisher(String, "vector_out", 10)

    def start_query(self, prompt):
        result = self.query_vector(prompt)
        print(str(result))
        self.start_publish(result)

    def start_publish(self, data):
        self.data = data
        self.timer = self.create_timer(
            self.get_parameter(
                'timer_exit').get_parameter_value().integer_value, 
            self.publish_stdin)

    def publish_stdin(self):
        """
        """
        if self.done: sys.exit(0)
        self.get_logger().debug(f"publish: {str(self.data)}")
        self.pub.publish(String(data=self.data))
        self.done = True

    def input_callback(self, msg):
        """
        """
        try:
            d = json.loads(msg.data)
            try:
                result = self.query_vector(
                    d['query'], 
                    limit = None if 'query_limit' not in d 
                        else int(d['query_limit'])
                )
                self.pub.publish(String(data=result))
            except Exception as e: 
                self.get_logger().error(
                    f'input_callback: {str(e)}')
        except:
            result = self.query_vector(msg.data)
            self.pub.publish(String(data=result))
       
    
    def init_qdrant(self):
        """
        """
        import logging
        from qdrant_client import QdrantClient

        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s %(name).15s %(levelname)-8s %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S")

        self.client = QdrantClient(self.query_url, prefer_grpc=True)
        self.client.set_model(self.query_model)

    def query_vector(self, prompt, limit=None):
        """
        """
        if not self.client: self.init_qdrant()

        results = self.client.query(
            collection_name = self.collection_name,
            query_text = prompt,
            limit = self.query_limit if not limit else limit
        )

        context = "\n".join(re.sub(r'event_[^ ]* [^ ]* ', '', r.document) for r in results)
        result = f"{prompt.strip()}\nContext:\n{context.strip()}\n\n"

        result = f"""
Context information is below.
---------------------
{context}
---------------------
Given the context information and not prior knowledge, answer the query.
Query: {prompt.strip()}
Answer:
"""

        self.get_logger().info(result)
        return result


# echo -e "var prompt=''\nDBQuery.shellBatchSize = 100000\nEJSON.stringify(db.memo_chat.find().toArray())" | mongosh --quiet mongodb://192.168.1.130/metadb 2>/dev/null | grep "^\["
# echo -e "var prompt=''\nDBQuery.shellBatchSize = 100000\nEJSON.stringify(db.memo_chat.find().toArray())" | mongosh --quiet mongodb://192.168.1.130/metadb 2>/dev/null | grep "^\[" | src/rosgpt4all/scripts/converter.py | jq . > x
# echo -e "var prompt=''\nDBQuery.shellBatchSize = 100000\nEJSON.stringify(db.memo_chat.find().limit(1).toArray())" | mongosh --quiet mongodb://192.168.1.130/metadb 2>/dev/null | grep "^\[" | src/rosgpt4all/scripts/converter.py --ros-args -r output:=/gpt/embed

def main():

    rclpy.init(args=None)
    node = VectorNode()

    if select.select([sys.stdin, ], [], [], 2.0)[0]:

        raw = sys.stdin.read()

        try:
            data = json.loads(raw)

            if isinstance(data, (list, dict)):

                documents = []
                if isinstance(data, dict):
                    documents = [data['data']]
                    data = [data]
                else:
                    for d in data:
                        documents.append(d['data'])

                result = {
                    'collection': node.collection_name,
                    'documents': documents,
                    'metadatas': data
                }
                
                node.start_publish(json.dumps(result))

        except Exception as e:
            node.start_query(raw)

    elif node.query:
        node.start_query(node.query)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
