# script for enabling and processing live lttng events to update latency calculations and UI

#!/usr/bin/env python3
import bt2
import subprocess
import time
import os
import sys

#!/usr/bin/env python3
import bt2
import subprocess
import sys

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

def wait_for_first_event_file(trace_dir, timeout=30):
  print(f"[*] Waiting for first event file in {trace_dir} to appear...")
  start = time.time()
  while True:
    for root, dirs, files in os.walk(trace_dir):
      for f in files:
        if f.endswith(".ctf"):  # UST event files have .ctf
          return True
    if time.time() - start > timeout:
      print(f"[!] Timeout waiting for first event file in {trace_dir}.")
      return False
    time.sleep(0.1)

def create_lttng_session(session_name, trace_dir, provider_name, tracepoint_name):
  wait_for_provider(PROVIDER_NAME)
  wait_for_first_event_file(TRACE_DIR)
  print(f"[*] Creating LTTng session {session_name}...")
  subprocess.run(["lttng", "create", session_name, "--output", trace_dir], check=True)
  subprocess.run(["lttng", "enable-event", "-u", f"{provider_name}:{tracepoint_name}"], check=True)
  subprocess.run(["lttng", "start", session_name], check=True)
  print(f"[*] LTTng session {session_name} started.")

def cleanup_session(session_name):
  print(f"[*] Stopping LTTng session {session_name}...")
  subprocess.run(["lttng", "stop", session_name])
  subprocess.run(["lttng", "destroy", session_name])
  print(f"[*] Session {session_name} cleaned up.")

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


def read_tracepoints():
  # Connect to live session using lttng-live plugin
  uri = f"lttng-live://127.0.0.1/{SESSION_NAME}?domain=ust"

  try:
    src = bt2.TraceCollectionMessageIterator(uri)
  except Exception as e:
    print(f"[!] Failed to open live trace: {e}")
    sys.exit(1)

  print("[*] Reading tracepoints...")
  for msg in src:
    if msg.type == bt2.MESSAGE_TYPE_EVENT:
      evt = msg.event
      print(f"[EVENT] {evt.name} @ {evt.timestamp_ns}")
      for field in evt.fields:
        print(f"  {field.name}: {field.value}")
  print("DONE")

if __name__ == "__main__":
  try:
    create_lttng_session(SESSION_NAME, TRACE_DIR, PROVIDER_NAME, TRACEPOINT_NAME)
    read_tracepoints()
  except KeyboardInterrupt:
    print("\n[*] Interrupted by user.")
  finally:
    cleanup_session(SESSION_NAME)
