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
import json
import json_embedder
from qdrant_client import QdrantClient
from typing import Any

class QdrantEmbedder(json_embedder.JsonEmbedder):
    """Qdrant JSON embedding interface.
    The __call__ function expects a string with a JSON dict containing the following attributes:
      collection: to be used collection name
      documents: list with document strings
      metadatas: list of metadata dictionaries
      ids: optional list of id strings, if not provided Qdrant will create them
    """

    def __init__(self, model: str = None, **kwargs: Any):
        if not len(kwargs):
            self.client = QdrantClient(host='localhost', port=6333)
        else:
            self.client = QdrantClient(**kwargs)
        self.client.set_model(model or 'nomic-ai/nomic-embed-text-v1')

    def __call__(self, j: str) -> None:
        data = json.loads(j)
        self.client.add(
            collection_name = data['collection'], 
            documents = data['documents'], 
            metadata = data['metadatas'], 
            ids = None if 'ids' not in data else data['ids'])
