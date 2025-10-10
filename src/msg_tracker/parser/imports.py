import bt2
from bt2 import event as bt2_event, field as bt2_field, trace_collection_message_iterator
import subprocess
import time
import os
import signal
import rclpy
import threading
import socket
import psutil
import getpass
import datetime
import atexit
import sys
import websocket
from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Empty
from datetime import datetime
from typing import Callable, Any
from collections import defaultdict, deque

TraceIterator = trace_collection_message_iterator.TraceCollectionMessageIterator
TraceEventMessage = bt2._EventMessageConst
TraceEvent = bt2_event._EventConst
TraceFields = bt2_field._StructureFieldConst