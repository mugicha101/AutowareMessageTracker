# script for enabling and processing live lttng events to update latency calculations and UI

#!/usr/bin/env python3
import bt2
import subprocess
import time
import os
import sys
import glob
import signal
import rclpy
import threading
import socket
import psutil
import getpass
import datetime
from rclpy.node import Node
from std_msgs.msg import Empty
from datetime import datetime

TRACE_DIR = "tmp/lttng-traces"
SESSION_NAME = "live_tracker_session"
PROVIDER_NAME = "tracker"
TRACEPOINT_NAME = "*"  # or specific tracepoint name

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

# def wait_for_first_event_file(trace_dir, timeout=30):
#   print(f"[*] Waiting for first event file in {trace_dir} to appear...")
#   start = time.time()
#   while True:
#     for root, dirs, files in os.walk(trace_dir):
#       for f in files:
#         if f.endswith(".ctf"):  # UST event files have .ctf
#           return True
#     if time.time() - start > timeout:
#       print(f"[!] Timeout waiting for first event file in {trace_dir}.")
#       return False
#     time.sleep(0.1)

def create_lttng_session(session_name, trace_dir, provider_name, tracepoint_name):
  remove_existing_session(SESSION_NAME)
  assert(wait_for_provider(PROVIDER_NAME))

  print(f"[*] Creating LTTng session {session_name}...")
  subprocess.run(["lttng", "create", session_name, "--live"], check=True)
  subprocess.run(["lttng", "enable-event", "-u", f"{provider_name}:{tracepoint_name}"], check=True)
  subprocess.run(["lttng", "start"], check=True)
  print(f"[*] LTTng session {session_name} started.")

  return True

def cleanup_session(session_name):
  print(f"[*] Stopping LTTng session {session_name}...")
  subprocess.run(["lttng", "stop"])
  subprocess.run(["lttng", "destroy"])
  print(f"[*] Session {session_name} cleaned up.")

  # wait a bit before creating again to prevent error
  time.sleep(0.5)

def remove_existing_session(session_name):
  # List existing sessions
  result = subprocess.run(
    ["lttng", "list", "sessions"],
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True,
  )
  if session_name in result.stdout:
    print(f"[*] Session '{session_name}' already exists. Destroying it...")
    cleanup_session(session_name)
  else:
    print(f"[*] Session '{session_name}' does not exist. Continuing...")

def ui_node_thread():
  rclpy.init()
  ui_node = rclpy.create_node("ui_node")
  ui_init_topic = "/msg_tracker/ui_init"
  ui_init_pub = ui_node.create_publisher(Empty, ui_init_topic, 10)

  def timer_callback():
    ui_init_pub.publish(Empty())
    print(f"[*] Published to {ui_init_topic}.")
  ui_node.create_timer(1.0, timer_callback)

  try:
    rclpy.spin(ui_node)
  except:
    pass
  finally:
    ui_node.destroy_node()
    try:
      rclpy.shutdown()
    except:
      pass

def launch_ui_node():
  print("[*] Launching UI Node...")
  thread = threading.Thread(target=ui_node_thread)
  thread.start()

def read_tracepoints():
  # wait_for_first_event_file(TRACE_DIR)

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
  while True:
    try:
      for msg in src:
        if type(msg) is bt2._EventMessageConst:
          event = msg.event
          timestamp = msg.default_clock_snapshot.ns_from_origin
          handle_event(event, timestamp)
    except bt2.TryAgain:
      pass

def handle_event(event, timestamp):
  print(f"[EVENT] {event.name} @ {datetime.fromtimestamp(timestamp / 1e9)}")

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
    launch_ui_node()
    create_lttng_session(SESSION_NAME, TRACE_DIR, PROVIDER_NAME, TRACEPOINT_NAME)
    read_tracepoints()
  except KeyboardInterrupt:
    print("\n[*] Interrupted by user.")
  finally:
    cleanup_session(SESSION_NAME)
