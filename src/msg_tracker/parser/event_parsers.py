from imports import *
from state_tracker import *

parser_map: dict[str, Callable[[TraceEventMessage], Any]] = {}

def parse_trace_event_message(msg: TraceEventMessage) -> Any:
  event = msg.event
  timestamp = msg.default_clock_snapshot.ns_from_origin
  field_str = "\n    " + "\n    ".join(f"{field}: {event[field]}" for field in event.payload_field)
  print(f"[EVENT {datetime.fromtimestamp(timestamp / 1e9)} {event.name}]{field_str}")
  name = event.name
  StateTracker.global_tracker.set_time(timestamp)
  ret = parser_map[name](event) if name in parser_map else None
  return ret

def trace_event_parser(name):
  def decorator(func):
    parser_map[name] = func
    return func
  return decorator

def norm_pub_id(pub_id: list[bytes]):
    return int.from_bytes(pub_id, "big")

@trace_event_parser("tracker:sub_init")
def sub_init(event: TraceEvent):
  sub_id = int(event["sub_id"])
  node = str(event["node"])
  topic = str(event["topic"])
  tracker = StateTracker.global_tracker
  tracker.sub_init(sub_id, node, topic)
  tracker.mapping_init_done = True

@trace_event_parser("tracker:pub_init")
def pub_init(event: TraceEvent):
  pub_id = norm_pub_id(event["pub_id"])
  node = str(event["node"])
  topic = str(event["topic"])
  tracker = StateTracker.global_tracker
  tracker.pub_init(pub_id, node, topic)
  tracker.mapping_init_done = True

@trace_event_parser("tracker:publish")
def publish(event: TraceEvent):
  cb_tid = int(event["cb_tid"])
  pub_id = norm_pub_id(event["pub_id"])
  stamp = int(event["stamp"])
  tracker = StateTracker.global_tracker
  cb = tracker.get_cb(cb_tid)
  msg = tracker.get_new_msg(pub_id, stamp)
  msg.parent = cb.direct_link_src
  msg.pub_time = tracker.time

@trace_event_parser("tracker:recieve")
def recieve(event: TraceEvent):
  cb_tid = int(event["cb_tid"])
  pub_id = norm_pub_id(event["pub_id"])
  stamp = int(event["stamp"])
  sub_id = int(event["sub_id"])
  tracker = StateTracker.global_tracker
  cb = tracker.get_cb(cb_tid)
  msg = tracker.get_msg(pub_id, stamp)
  msg.rec_time = tracker.time
  msg.sub_id = sub_id
  tracker.add_recieved_msg(msg.pub_id, msg.stamp)

  # msg can be None if publish not captured in which case treat direct links from message as root messages
  cb.direct_link_src = (msg.pub_id, msg.stamp)

@trace_event_parser("tracker:no_recieve")
def no_recieve(event: TraceEvent):
  cb_tid = int(event["cb_tid"])
  tracker = StateTracker.global_tracker
  cb = tracker.get_cb(cb_tid)
  cb.direct_link_src = None

@trace_event_parser("tracker:indirect_link")
def indirect_link(event: TraceEvent):
  tracker = StateTracker.global_tracker
  # TODO
