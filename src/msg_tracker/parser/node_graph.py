import dash
from dash import html, dcc, Output, Input
import dash_cytoscape as cyto
import threading
from collections import defaultdict
from enum import Enum

# TODO: setup foxglove node graph system

class NodeType(Enum):
    NODE = 0
    TOPIC = 1
    PUB = 2
    SUB = 3
    TIMER = 4

class Node:
  def __init__(self, id, type: NodeType, name, x = 0.0, y = 0.0):
    self.id = id
    self.type = type
    self.name = name
    self.x = x
    self.y = y

  def publish(self):
    marker = { "id": self.id }

class Edge:
  def __init__(self, src, dst, weight = 1, label = ""):
    self.src = src
    self.dst = dst
    self.weight = weight
    self.label = label

class GraphUI:
  def __init__(self):
    self.nodes = {}
    self.edges = {}

  def add_node(self, type: NodeType, name):
    assert(name not in self.nodes)
    self.nodes[name] = Node(len(self.nodes), type, name)

  def set_edge(self, src, dst, label="", weight=1):
    # TODO
    pass

if __name__ == "__main__":
  g = GraphUI()
  g.add_node("A")
  g.add_node("B")
  g.add_edge("A", "B")
