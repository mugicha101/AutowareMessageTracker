# script for enabling and processing live lttng events to update latency calculations and UI

#!/usr/bin/env python3
from imports import *
from state_tracker import *
from trace_parser import *
from event_parsers import *

SESSION_NAME = "live_tracker_session"
PROVIDER_NAME = "tracker"
TRACEPOINT_NAME = "*"  # or specific tracepoint name
UPDATE_PERIOD = 100_000 # microseconds between babeltrace updates 

def wait_for_provider(provider_name, timeout=30):
  print(f"[*] Waiting for provider '{provider_name}' to appear...")
  start = time.time()
  while True:
    result = subprocess.run(
      ["lttng", "list", "-u"],
      stdout=subprocess.PIPE,
      stderr=subprocess.PIPE,
      text=True,
    )
    if provider_name in result.stdout:
      print(f"[+] Provider '{provider_name}' detected.")
      return True
    if time.time() - start > timeout:
      print(f"[!] Timeout waiting for provider '{provider_name}'.")
      return False
    time.sleep(0.5)

def create_lttng_session(session_name, provider_name, tracepoint_name):
  assert(wait_for_provider(PROVIDER_NAME))

  print(f"[*] Creating LTTng session {session_name}...")
  while True:
    cleanup_session(SESSION_NAME)
    res = subprocess.run(["lttng", "create", session_name, f"--live={UPDATE_PERIOD}"])
    if res.returncode == 0:
      break
    print("[!] LTTNG Create Failed, retrying...")
    time.sleep(0.5)

  subprocess.run(["lttng", "enable-event", "-u", f"{provider_name}:{tracepoint_name}"], check=True)
  subprocess.run(["lttng", "start"], check=True)
  print(f"[*] LTTng session {session_name} started.")

  return True

def cleanup_session(session_name):
  print(f"[*] Stopping LTTng session {session_name}...")
  subprocess.run(["lttng", "stop"])
  subprocess.run(["lttng", "destroy"])
  print(f"[*] Session {session_name} cleaned up.")

def mapping_node_thread():
  rclpy.init()
  mapping_node = rclpy.create_node("mapping_node")
  mapping_init_topic = "/msg_tracker/mapping_init"
  mapping_init_pub = mapping_node.create_publisher(Empty, mapping_init_topic, 10)

  timer = None
  def timer_callback():
    if timer is not None and StateTracker.global_tracker is not None and StateTracker.global_tracker.mapping_init_done:
      timer.cancel() # TODO: handle new nodes entering system (maybe just keep the timer running)
    mapping_init_pub.publish(Empty())
    print(f"[*] Published to {mapping_init_topic}.")
  timer = mapping_node.create_timer(1.0, timer_callback)

  try:
    rclpy.spin(mapping_node)
  except:
    pass
  finally:
    mapping_node.destroy_node()
    try:
      rclpy.shutdown()
    except:
      pass

def launch_mapping_node():
  print("[*] Launching UI Node...")
  thread = threading.Thread(target=mapping_node_thread)
  thread.start()

def read_tracepoints():
  # Connect to live session using lttng-live plugin
  uri = f"net://localhost/host/{socket.gethostname()}/{SESSION_NAME}"

  # taken from https://github.com/ros2/ros2_tracing/issues/149
  src = bt2.TraceCollectionMessageIterator(
    bt2.ComponentSpec.from_named_plugin_and_component_class('ctf', 'lttng-live', {
        # See:
        #   https://babeltrace.org/docs/v2.0/man7/babeltrace2-source.ctf.lttng-live.7/#doc-_initialization_parameters
        'inputs': [uri],
    }),
  )

  # event loop
  # again taken from https://github.com/ros2/ros2_tracing/issues/149
  print("[*] Reading tracepoints...")
  StateTracker.global_tracker = StateTracker()
  while True:
    try:
      for msg in src:
        if type(msg) is bt2._EventMessageConst:
          event = msg.event
          timestamp = msg.default_clock_snapshot.ns_from_origin
          parse_trace_event_message(msg)
    except bt2.TryAgain:
      pass

def setup_lttng():
  # spawn relay daemon if none exist already
  try:
    result = subprocess.run(["pidof", "lttng-relayd"], capture_output=True, text=True)
    pids = [int(pid) for pid in result.stdout.strip().split()]
  except Exception:
    pids = []
  print(pids)
  if not pids:
    subprocess.Popen(["lttng-relayd", "-d"])

  # # kill any zombie viewers
  # viewer_sockets = glob.glob("/tmp/lttng-viewer-*")
  # for sock in viewer_sockets:
  #   try:
  #     os.remove(sock)
  #     print(f"Removed stuck viewer socket: {sock}")
  #   except FileNotFoundError:
  #     pass
  #   except PermissionError:
  #     print(f"Permission denied when removing: {sock}")

  # kill zombie python processes
  current_user = getpass.getuser()
  target_cmd_prefix = "python3 src/msg_tracker"
  my_pid = os.getpid() 

  for proc in psutil.process_iter(['pid', 'name', 'username', 'cmdline']):
    try:
      if proc.info['username'] != current_user:
        continue

      cmdline = " ".join(proc.info['cmdline'])
      if cmdline.startswith(target_cmd_prefix):
        pid = proc.info['pid']
        if pid == my_pid:
          continue

        print(f"Killing Parser PID: {pid}, CMD: {cmdline}")
        os.kill(pid, signal.SIGUSR1)
        proc.wait(timeout=1)
        if proc.is_running():
          print(f"Parser PID {pid} still alive, terminating...")
          proc.terminate()
          proc.wait(timeout=3)
        print(f"Parser PID {pid} stopped.")
    except (psutil.NoSuchProcess, psutil.AccessDenied):
      continue


if __name__ == "__main__":
  try:
    setup_lttng()
    launch_mapping_node()
    create_lttng_session(SESSION_NAME, PROVIDER_NAME, TRACEPOINT_NAME)
    read_tracepoints()
  except KeyboardInterrupt:
    print("\n[*] Interrupted by user.")
  finally:
    cleanup_session(SESSION_NAME)
