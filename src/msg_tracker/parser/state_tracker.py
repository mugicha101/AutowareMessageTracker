from imports import *
from node_graph import *

INF = 10000000000

class DataDistr:
  def __init__(self):
    self.points = []
    self.min_val = INF
    self.max_val = -INF
    self.sum = 0

  def add_point(self, x):
    self.points.append(x)
    self.min_val = min(self.min_val, x)
    self.max_val = max(self.max_val, x)
    self.sum += x
    self.avg_val = 0 if len(self.points) == 0 else self.sum / len(self.points)

class CallbackState:
  def __init__(self):
    self.direct_link_src = None

class MessageState:
  def __init__(self, pub_id, stamp):
    self.pub_id = pub_id
    self.stamp = stamp
    self.sub_id = None
    self.pub_time = None
    self.rec_time = None
    self.parent = None

class EntityState:
  def __init__(self, name):
    self.name = name
    self.outgoing: set[int] = set()
    self.incoming: set[int] = set()

class NodeState(EntityState):
  def __init__(self, name):
    super().__init__(name)

class TopicState(EntityState):
  def __init__(self, name):
    super().__init__(name)

class PortState:
  def __init__(self):
    self.node = None
    self.topic = None

class PubSubPair:
  def __init__(self, pub_id, sub_id):
    self.pub_id = pub_id
    self.sub_id = sub_id
    self.latencies = DataDistr()

class PubState(PortState):
  def __init__(self):
    super().__init__()

class SubState(PortState):
  def __init__(self):
    super().__init__()

class StateTracker:
  global_tracker: "StateTracker" = None
  
  def __init__(self):
    self.time = 0
    self.mapping_init_done = False
    self.cb_map: dict[int, CallbackState] = defaultdict(lambda : CallbackState())
    self.msg_map: dict[tuple[int,int], MessageState] = {}
    self.node_map: dict[str,NodeState] = {}
    self.topic_map: dict[str,TopicState] = {}
    self.pub_map: dict[int,PubState] = defaultdict(lambda : PubState())
    self.sub_map: dict[int,PubState] = defaultdict(lambda : SubState())
    self.pair_map: dict[tuple[int,int],PubSubPair] = {}
    self.anal_queue: deque[tuple[int,int]] = deque() # recieved messages that haven't been analyzed
    self.ui_graph = GraphUI()
    self.ui_graph.run(use_thread=True)

  def set_time(self, time):
    assert(self.time <= time)
    self.time = time

  # get callback state (creates if none exists)
  def get_cb(self, cb_tid: int):
    return self.cb_map[cb_tid]
  
  # create a message state for a new message (error if already exists)
  def get_new_msg(self, pub_id: int, stamp):
    id = (pub_id, stamp)
    assert(id not in self.msg_map)
    msg = MessageState(*id)
    self.msg_map[id] = msg
    return msg

  # get an existing message (None if doesn't exist)
  def get_msg(self, pub_id: int, stamp):
    id = (pub_id, stamp)
    return self.msg_map.get(id)
  
  # get node state (creates if none exists)
  def get_node(self, node_name):
    if node_name not in self.node_map:
      self.node_map[node_name] = NodeState(node_name)
      self.ui_graph.set_node(node_name)
    return self.node_map[node_name]
  
  # get topic state (creates if none exists)
  def get_topic(self, topic_name):
    if topic_name not in self.topic_map:
      self.topic_map[topic_name] = TopicState(topic_name)
    return self.topic_map[topic_name]

  # get publisher state (create if none exists)
  def get_pub(self, pub_id: int):
    return self.pub_map[pub_id]
  
  # get subscriber state (create if none exists)
  def get_sub(self, sub_id: int):
    return self.sub_map[sub_id]
  
  # get pubsub pair (create if none exists)
  def get_pair(self, pub_id: int, sub_id: int):
    id = (pub_id, sub_id)
    if id not in self.pair_map:
      self.pair_map[id] = PubSubPair(pub_id, sub_id)
      pub = self.get_pub(pub_id)
      sub = self.get_sub(sub_id)
      assert(pub.topic == sub.topic)
    return self.pair_map[id]
  
  # link pub node topic
  def pub_init(self, pub_id, node_name, topic_name):
    pub = self.get_pub(pub_id)
    pub.node = node_name
    pub.topic = topic_name
    node = self.get_node(node_name)
    node.outgoing.add(pub_id)
    topic = self.get_topic(topic_name)
    topic.incoming.add(pub_id)

  # link sub node topic
  def sub_init(self, sub_id, node_name, topic_name):
    sub = self.get_sub(sub_id)
    sub.node = node_name
    sub.topic = topic_name
    node = self.get_node(node_name)
    node.incoming.add(sub_id)
    topic = self.get_topic(topic_name)
    topic.outgoing.add(sub_id)
  
  def add_recieved_msg(self, pub_id, stamp):
    self.anal_queue.append((pub_id, stamp))
    self.analyze()
  
  # consume analysis queue to update analysis
  # requires all nodes to be known (via sub_init and pub_init)
  def analyze(self):
    if not self.mapping_init_done:
      return
    
    while len(self.anal_queue) > 0:
      pub_id, stamp = self.anal_queue.popleft()
      msg = self.get_msg(pub_id, stamp)
      pair = self.get_pair(pub_id, msg.sub_id)
      lat = pair.latencies
      lat.add_point(msg.rec_time - msg.pub_time)
      pub = self.get_pub(pub_id)
      sub = self.get_sub(msg.sub_id)
      self.ui_graph.set_edge(pub.node, sub.node, f"{pub.topic}: [{lat.min_val},{lat.avg_val},{lat.max_val}]ns")
      print(f"MSG {msg.rec_time - msg.pub_time}")
