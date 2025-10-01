from imports import *
from state_tracker import *

parser_map: dict[str, Callable[[TraceEventMessage], Any]] = {}

def utf8(x):
  return str(x).encode('utf-8', errors='replace').decode('utf-8')

def parse_trace_event_message(msg: TraceEventMessage) -> Any:
  event = msg.event
  timestamp = msg.default_clock_snapshot.ns_from_origin
  field_str = "\n    " + "\n    ".join(f"{field}: {utf8(event[field])}" for field in event.payload_field)
  # print(f"[EVENT {utf8(datetime.fromtimestamp(timestamp / 1e9))} {utf8(event.name)}]{field_str}")
  StateTracker.global_tracker.set_time(timestamp)
  name = event.name
  ret = parser_map[name](event) if name in parser_map else None
  return ret

def trace_event_parser(name):
  def decorator(func):
    parser_map[name] = func
    return func
  return decorator

def norm_pub_id(pub_id: list[bytes]):
    # due to pub_id being different for the same publisher across different nodes
    return int.from_bytes(pub_id, "big")

@trace_event_parser("tracker:sub_init")
def sub_init(event: TraceEvent):
  tracker = StateTracker.global_tracker
  sub_id = int(event["sub_id"])
  node = str(event["node"])
  topic = str(event["topic"])
  tracker.sub_init(sub_id, node, topic)
  tracker.mapping_init_done = True

@trace_event_parser("tracker:pub_init")
def pub_init(event: TraceEvent):
  tracker = StateTracker.global_tracker
  pub_id = norm_pub_id(event["pub_id"])
  node = str(event["node"])
  topic = str(event["topic"])
  tracker.pub_init(pub_id, node, topic)
  tracker.mapping_init_done = True

@trace_event_parser("tracker:publish")
def publish(event: TraceEvent):
  tracker = StateTracker.global_tracker
  if not tracker.mapping_init_done:
    return

  cb_tid = int(event["cb_tid"])
  pub_id = norm_pub_id(event["pub_id"])
  stamp = int(event["stamp"])
  cb = tracker.get_cb(cb_tid)
  if (pub_id, stamp) in tracker.msg_map:
    # indirect link
    msg = tracker.get_msg(pub_id, stamp)
  else:
    # direct link
    msg = tracker.get_new_msg(pub_id, stamp)
    msg.parent = cb.direct_link_src
  msg.pub_time = tracker.time

@trace_event_parser("tracker:recieve")
def recieve(event: TraceEvent):
  tracker = StateTracker.global_tracker
  if not tracker.mapping_init_done:
    return
  
  cb_tid = int(event["cb_tid"])
  stamp = int(event["stamp"])
  sub_id = int(event["sub_id"])
  sub_pub_id = norm_pub_id(event["pub_id"])
  cb = tracker.get_cb(cb_tid)
  pub_id = tracker.find_recv_msg_pub_id(sub_id, stamp - 1)
  tracker.add_pub_id_alias(pub_id, sub_pub_id)
  msg = tracker.get_msg(pub_id, stamp - 1)
  msg.rec_time = tracker.time
  msg.sub_id = sub_id
  tracker.add_recieved_msg(msg.pub_id, msg.stamp)

  # msg can be None if publish not captured in which case treat direct links from message as root messages
  cb.direct_link_src = (msg.pub_id, msg.stamp)

@trace_event_parser("tracker:no_recieve")
def no_recieve(event: TraceEvent):
  tracker = StateTracker.global_tracker
  if not tracker.mapping_init_done:
    return
  
  cb_tid = int(event["cb_tid"])
  cb = tracker.get_cb(cb_tid)
  cb.direct_link_src = None

@trace_event_parser("tracker:indirect_link")
def indirect_link(event: TraceEvent):
  tracker = StateTracker.global_tracker
  if not tracker.mapping_init_done:
    return
  
  prev_pub_id = norm_pub_id(event["prev_pub_id"])
  prev_stamp = int(event["prev_stamp"])
  next_pub_id = norm_pub_id(event["next_pub_id"])
  next_stamp = int(event["next_stamp"])
  prev_msg = tracker.get_msg(prev_pub_id, prev_stamp)
  next_msg = tracker.get_new_msg(next_pub_id, next_stamp)
  next_msg.parent = prev_msg
  
