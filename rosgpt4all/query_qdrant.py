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

import logging
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name).15s %(levelname)-8s %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S")

client = QdrantClient("http://192.168.1.134:6333", prefer_grpc=True)
#client.set_model("fast-nomic-embed-text-v1")
client.set_model("nomic-ai/nomic-embed-text-v1")

#client.get_collections()

#client.add(
#    collection_name="knowledge-base",
#    documents=[
#        "Qdrant is a vector database & vector similarity search engine. It deploys as an API service providing search for the nearest high-dimensional vectors. With Qdrant, embeddings or neural network encoders can be turned into full-fledged applications for matching, searching, recommending, and much more!",
#        "Docker helps developers build, share, and run applications anywhere — without tedious environment configuration or management.",
#        "PyTorch is a machine learning framework based on the Torch library, used for applications such as computer vision and natural language processing.",
#        "MySQL is an open-source relational database management system (RDBMS). A relational database organizes data into one or more data tables in which data may be related to each other; these relations help structure the data. SQL is a language that programmers use to create, modify and extract data from the relational database, as well as control user access to the database.",
#        "NGINX is a free, open-source, high-performance HTTP server and reverse proxy, as well as an IMAP/POP3 proxy server. NGINX is known for its high performance, stability, rich feature set, simple configuration, and low resource consumption.",
#        "FastAPI is a modern, fast (high-performance), web framework for building APIs with Python 3.7+ based on standard Python type hints.",
#        "SentenceTransformers is a Python framework for state-of-the-art sentence, text and image embeddings. You can use this framework to compute sentence / text embeddings for more than 100 languages. These embeddings can then be compared e.g. with cosine-similarity to find sentences with a similar meaning. This can be useful for semantic textual similar, semantic search, or paraphrase mining.",
#        "The cron command-line utility is a job scheduler on Unix-like operating systems. Users who set up and maintain software environments use cron to schedule jobs (commands or shell scripts), also known as cron jobs, to run periodically at fixed times, dates, or intervals.",
#    ]
#)

#prompt = """
#What tools should I need to use to build a web service using vector embeddings for search?
#"""

prompt = "what about music"

results = client.query(
    collection_name="memo_embedder",
    #embeddings="nomic-embed-text-v1",
    query_text=prompt,
    query_filter=Filter(
        must=[
            FieldCondition(key="metadata[].key", match=MatchValue(value="user_id")),
            FieldCondition(key="metadata[].value", match=MatchValue(value="585559779"))
        ]
    ),
    limit=3,
)

print(results)

context = "\n".join(r.document for r in results)

metaprompt = f"""
{prompt.strip()}

### Context:
{context.strip()}

"""

# Look at the full metaprompt
print(metaprompt)
