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
from typing import Any
import chromadb
from chromadb.utils import embedding_functions as ef
from gpt4all import GPT4All
from gpt4all import Embed4All

class GPT4AllEmbeddingFunction(chromadb.EmbeddingFunction):
    """Basic Embed4All chromadb.EmbeddingFunction
    """
    def __call__(self, input: chromadb.Documents) -> chromadb.Embeddings:
        embedder = Embed4All()
        return [embedder.embed(chunk) for chunk in input]

class ChromaEmbedder(json_embedder.JsonEmbedder):
    """Chroma JSON embedding interface.
    The __call__ function expects a string with a JSON dict containing the following attributes:
      collection: to be use collection name
      documents: list with document strings
      metadatas: list of metadata dictionaries
      ids: list of id strings
    """

    def __init__(self, model: str = None, **kwargs: Any):
        if not len(kwargs):
            self.client = chromadb.HttpClient(host="localhost", port="8000")
        else:
            if 'path' in kwargs:
                self.client = chromadb.PersistentClient(path=kwargs['path'])
            else:
                self.client = chromadb.HttpClient(**kwargs)
        self.ef = ef.SentenceTransformerEmbeddingFunction(
            model_name=model or "all-MiniLM-L6-v2")

    def __call__(self, j: str) -> None:
        data = json.loads(j)
        collection = self.client.get_or_create_collection(
            name = data['collection'], 
            embedding_function = self.ef)
        collection.add(
            documents = data['documents'],
            metadatas = data['metadatas'],
            ids = data['ids'])
