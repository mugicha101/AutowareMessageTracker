from imports import *
from node_graph import *

INF = 10000000000
MAX_LATENCY_NS = 5 * 100000000 # max time to hold in transit messages for (in ns)

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
  def __init__(self, pub_id, seq_num):
    self.pub_id = pub_id
    self.seq_num = seq_num
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
    self.sent_msg_pub_id: dict[int,int] = {} # seq_num -> pub_id for messages in transit
    self.sent_msg_recieved: dict[int,int] = {} # seq_num -> recieved
    self.sent_msgs: deque[tuple[int,int,int]] = deque() # list (pub_time, pub_id, seq_num), assumes recieved in pub_time order

  def add_msg(self, pub_time, pub_id, seq_num):
    # delete messages older than MAX_LATENCY
    while len(self.sent_msgs) > 0 and self.sent_msgs[0][0] + MAX_LATENCY_NS > pub_time:
      if not self.sent_msg_recieved[self.sent_msgs[0][2]]:
        print(f"WARNING - message never recieved: pub_time={self.sent_msgs[0][0]}, pub_id={self.sent_msgs[0][1]}, seq_num={self.sent_msgs[0][2]}")
      del self.sent_msg_pub_id[self.sent_msgs[0][2]]
      del self.sent_msg_recieved[self.sent_msgs[0][2]]
      self.sent_msgs.popleft()
    self.sent_msg_pub_id[seq_num] = pub_id
    self.sent_msg_recieved[seq_num] = False
    self.sent_msgs.append((pub_time, pub_id, seq_num))

  def find_msg_pub_id(self, seq_num):
    pub_id = self.sent_msg_pub_id.get(seq_num)
    if pub_id is None:
      return None
    
    self.sent_msg_recieved[seq_num] = True
    return pub_id

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
  def __init__(self, pub_id):
    super().__init__()
    self.pub_id = pub_id

class SubState(PortState):
  def __init__(self, sub_id):
    super().__init__()
    self.sub_id = sub_id

class StateTracker:
  global_tracker: "StateTracker" = None
  
  def __init__(self):
    self.time = 0
    self.mapping_init_done = False
    self.cb_map: dict[int, CallbackState] = defaultdict(lambda : CallbackState())
    self.msg_map: dict[tuple[int,int], MessageState] = {}
    self.node_map: dict[str,NodeState] = {}
    self.topic_map: dict[str,TopicState] = {}
    self.pub_map: dict[int,PubState] = {}
    self.sub_map: dict[int,PubState] = {}
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
  # also puts on topic
  def get_new_msg(self, pub_id: int, seq_num):
    id = (pub_id, seq_num)
    assert(id not in self.msg_map)
    msg = MessageState(*id)
    self.msg_map[id] = msg
    self.get_topic(self.get_pub(pub_id).topic).add_msg(self.time, pub_id, seq_num)
    return msg
  
  # figure out recieved msg's pub_id from sub_id, seq_num
  def find_recv_msg_pub_id(self, sub_id, seq_num):
    return self.get_topic(self.get_sub(sub_id).topic).find_msg_pub_id(seq_num)
  
  # maps subscriber's publisher id to original publisher id
  # if they already match, does nothing
  def add_pub_id_alias(self, canonical_pub_id, alias_pub_id):
    if alias_pub_id in self.pub_map:
      return
    
    self.pub_map[alias_pub_id] = self.pub_map[canonical_pub_id]

  # get an existing message (None if doesn't exist)
  def get_msg(self, pub_id: int, seq_num):
    id = (self.get_pub(pub_id).pub_id, seq_num)
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
    if pub_id not in self.pub_map:
      self.pub_map[pub_id] = PubState(pub_id)
    return self.pub_map[pub_id]
  
  # get subscriber state (create if none exists)
  def get_sub(self, sub_id: int):
    if sub_id not in self.sub_map:
      self.sub_map[sub_id] = SubState(sub_id)
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
  
  def add_recieved_msg(self, pub_id, seq_num):
    pub = self.get_pub(pub_id)
    pub_id = pub.pub_id
    msg = self.get_msg(pub_id, seq_num)
    sub = self.get_sub(msg.sub_id)
    pair = self.get_pair(pub_id, msg.sub_id)
    lat = pair.latencies
    lat.add_point(msg.rec_time - msg.pub_time)
    self.ui_graph.set_edge(pub.node, sub.node, f"{pub.topic}: [{lat.min_val},{lat.avg_val},{lat.max_val}]ns")
    print(f"MSG {msg.rec_time - msg.pub_time} NMSG: {len(self.msg_map)}")
