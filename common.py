#!/usr/bin/env python

# Copyright (c) 2022  Carnegie Mellon University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import time
import json
import threading
import traceback
import logging
import subprocess
from uuid import UUID
from collections import deque

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.time import Time

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
from cabot_msgs.srv import Speak
from cabot_msgs.msg import Log
from std_msgs.msg import String, Int16
from diagnostic_msgs.msg import DiagnosticArray

from cabot import util
from cabot.event import BaseEvent
from cabot_ui.event import NavigationEvent
from cabot_ace import BatteryDriverDelegate
from cabot_log_report import LogReport

CABOT_BLE_VERSION = "20230222"

ble_manager = None

# settings for roslibpy reconnection
rclpy.init()

ROS_CLIENT_CONNECTED = [False]
DEBUG=False

### Debug
def set_debug_mode():
    from logging import StreamHandler, Formatter

    for key in logging.common.logger.manager.common.loggerDict:
        #for key in ["pygatt.device"]:
        try:
            logging.common.logger.manager.common.loggerDict[key].setLevel(logging.DEBUG)
        except:
            pass

if DEBUG:
    set_debug_mode()



class CaBotNode_Pub(Node):
    def __init__(self):
        super().__init__('cabot_node_pub')
    
        self.cabot_event_pub = self.create_publisher(String, '/cabot/event', 5)
        self.ble_hb_pub = self.create_publisher(String, '/cabot/ble_heart_beat', 5)
        self.activity_log_pub = self.create_publisher(Log, '/cabot/activity_log', 5)
    def cabot_pub_event(self, msg):
        self.cabot_event_pub.publish(msg)
    def cabot_ble_hb_pub(self, msg):
        self.ble_hb_pub.publish(msg)
    def cabot_activity_log_pub(self, log_msg):
        self.activity_log_pub.publish(log_msg)

#class CaBotNode_Sub(Node):
#    def __init__(self):
#        super().__init__('cabot_node_sub')
    
#        self.diagnostics_sub = self.create_subscription(DiagnosticArray, "/diagnostics_agg", diagnostic_agg_callback)
#        self.cabot_event_sub = self.create_subscription(String, '/cabot/event', cabot_event_callback)
#        self.cabot_touch_sub = self.create_subscription(Int16, '/cabot/touch', cabot_touch_callback)
#        self.speak_service = self.create_service(Speak, '/speak')
#        self.restart_localization_service = self.create_service(mf_localization_msgs/RestartLocalization, '/restart_localization')

message_buffer = deque(maxlen=10)

node_pub = CaBotNode_Pub()

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

def activity_log(category="", text="", memo=""):
    now = Clock(clock_type=ClockType.ROS_TIME).now()
    logger.info("category={}, text={}, memo={}".format(category, text, memo))
    try:
        log_msg = Log()
        log_msg.header.stamp = Time(seconds=now.seconds_nanoseconds()[0], nanoseconds=now.seconds_nanoseconds()[1]).to_msg()
        log_msg.category = category
        log_msg.text = text
        log_msg.memo = memo

        node_pub.cabot_activity_log_pub(log_msg)
    except:
        logger.info(traceback.format_exc())


diagnostics = []
def diagnostic_agg_callback(msg):
    global diagnostics
    diagnostics = msg.status
    for diagnostic in diagnostics:
        # reduce floating number digits
        for i in range(len(diagnostic.values)-1, -1, -1):
            value = diagnostic.values[i]
            if value['key'] == 'Minimum acceptable frequency (Hz)' or \
               value['key'] == 'Maximum acceptable frequency (Hz)' or \
               value['key'] == 'Events in window' or \
               value['key'] == 'Events since startup':
                diagnostic['values'].pop(i)
                continue
            try:
                value.value = "%.2f"%(float(value.value))
            except:
                pass

event_handlers = []
def add_event_handler(handler):
    global event_handlers
    if not handler in event_handlers:
        event_handlers.append(handler)

def remove_event_handler(handler):
    global event_handlers
    event_handlers.remove(handler)

def clear_event_handler():
    global event_handlers
    event_handlers.clear()

def cabot_event_callback(msg):
    logger.info("cabot_event_callback is called")
    global event_handlers
    if event_handlers.count == 0:
        logger.error("There is no event_handler instance")

    request_id = time.clock_gettime_ns(time.CLOCK_REALTIME)
    for handler in event_handlers:
        handler.handleEventCallback(msg, request_id)
    activity_log("cabot/event", msg.data)

def cabot_touch_callback(msg):
    message_buffer.append(msg.data)


message_buffer = None

@util.setInterval(0.2)
def send_touch():
    if message_buffer:
        message = message_buffer.pop()
    else:
        message = -1
    
    global event_handlers
    if event_handlers.count == 0:
        logger.error("There is no event_handler instance")

    for handler in event_handlers:
        handler.handleTouchCallback(message)

send_touch()


class BLESubChar:
    def __init__(self, owner, uuid, indication=False):
        self.owner = owner
        self.uuid = uuid
        self.indication = indication
        self.valid = False
        self.target = None

    def callback(self, handle, value):
        raise RuntimeError("callback is not implemented")

    def not_found(self):
        pass

    def subscribe_to(self, target):
        self.target = target
        try:
            target.subscribe(self.uuid, self.callback, indication=self.indication)
            self.valid = True
        except:
            logger.error(traceback.format_exc())
            logger.info("could not connect to char %s", self.uuid)


class CabotLogChar(BLESubChar):
    def __init__(self, owner, uuid, manager):
        super().__init__(owner, uuid)
        self.manager = manager

    def callback(self, handle, value):
        value = value.decode("utf-8")
        try:
            data = json.loads(value)
            activity_log(**data)
        except:
            activity_log("ble", value, "")

    def not_found(self):
        logger.error("%s is not implemented", self.uuid)


class CabotManageChar(BLESubChar):
    def __init__(self, owner, uuid, manager):
        super().__init__(owner, uuid)
        self.manager = manager

    def callback(self, handle, value):
        value = value.decode("utf-8")
        if value == "reboot":
            self.manager.rebootPC()
        if value == "poweroff":
            self.manager.poweroffPC()
        if value == "stop":
            self.manager.stopCaBot()
        if value == "start":
            self.manager.startCaBot()
        if value.startswith("lang"):
            lang = value[5:]
            event = NavigationEvent(subtype="language", param=lang)
            msg = String()
            msg.data = str(event)
            node_pub.cabot_pub_event(msg)
#       if value.startswith("restart_localization"):
#           def callback(response):
#           logger.info(f"Localization restart: {response=}")
#           request = roslibpy.ServiceRequest({})
#           restart_localization_service.call(request, callback)

    def not_found(self):
        logger.error("%s is not implemented", self.uuid)


class DestinationChar(BLESubChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def callback(self, handle, value):
        value = value.decode("utf-8")
        logger.info("destination_callback %s", value)

        if value == "__cancel__":
            logger.info("cancel navigation")
            event = NavigationEvent(subtype="cancel", param=None)
            msg = String()
            msg.data = str(event)
            node_pub.cabot_pub_event(msg)
            return

        logger.info("destination: %s", value)
        event = NavigationEvent(subtype="destination", param=value)
        msg = String()
        msg.data = str(event)
        node_pub.cabot_pub_event(msg)


class SummonsChar(BLESubChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def callback(self, handle, value):
        value = value.decode("utf-8")
        logger.info("summons_callback %s", value)
        event = NavigationEvent(subtype="summons", param=value)
        msg = String()
        msg.data = str(event)
        node_pub.cabot_pub_event(msg)


class HeartbeatChar(BLESubChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def callback(self, handle, value):
        value = value.decode("utf-8")
        logger.info("heartbeat(%s):%s", self.owner.address, value)
        msg = String()
        msg.data = value
        node_pub.cabot_ble_hb_pub(msg)
        self.owner.last_heartbeat = time.time()


class StoreChar(BLESubChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def callback(self, handle, value):
        value = value.decode("utf-8")
        logger.info("store(%s):%s", self.owner.address, value)


class BLENotifyChar:
    def __init__(self, owner, uuid):
        self.owner = owner
        self.uuid = uuid

    def send_text(self, uuid, text, priority=10):
        self.owner.send_text(uuid, text, priority)


class VersionChar(BLENotifyChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)
        self.version = CABOT_BLE_VERSION

    def notify(self):
        logger.info("sending version")
        self.send_text(self.uuid, self.version)


class StatusChar(BLENotifyChar):
    def __init__(self, owner, uuid, func, interval=5):
        super().__init__(owner, uuid)
        self.func = func
        self.interval = interval
        self.count = 0
        self.loopStop = None

    def start(self):
        self.loopStop = self._loop()

    @util.setInterval(1)
    def _loop(self):
        self.count += 1
        if self.interval <= self.count:
            self.count = 0
            self.notify()

    def notify(self):
        if self.func():
            status = json.dumps(self.func().json, separators=(',', ':'))
            self.send_text(self.uuid, status)

    def stop(self):
        if self.loopStop:
            self.loopStop.set()


class SpeakChar(BLENotifyChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def handleSpeak(self, req):
        if not self.owner.ready:
            return None

        jsonText = json.dumps(req, separators=(',', ':'))
        self.send_text(self.uuid, jsonText, priority=0)
        activity_log("ble speech request", jsonText)
        return True


class EventChars(BLENotifyChar):
    def __init__(self, owner, navi_uuid):
        super().__init__(owner, None) # uuid is not set because EventChars uses multiple uuids.
        self.navi_uuid = navi_uuid

    def handleEventCallback(self, msg, request_id):
        event = BaseEvent.parse(msg['data'])
        if event is None:
            logger.error("cabot event %s cannot be parsed", msg['data'])
            return

        if event.type != NavigationEvent.TYPE:
            return

        if event.subtype not in ["next", "arrived", "content", "sound", "getlanguage"]:
            return
        req = {
            'request_id': request_id,
            'type': event.subtype,
            'param': event.param if event.param else ""
        }
        jsonText = json.dumps(req, separators=(',', ':'))
        self.send_text(self.navi_uuid, jsonText)

class TouchChars(BLENotifyChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, None) # uuid is not set because EventChars uses multiple uuids.
        self.uuid = uuid

    def handleTouchCallback(self, msg):
        req = {
            'level': msg
        }
        jsonText = json.dumps(req, separators=(',', ':'))
        self.send_text(self.uuid, jsonText)


class CabotLogRequestChar(BLESubChar):
    def __init__(self, owner, uuid, manager, response_char):
        super().__init__(owner, uuid)
        self.manager = manager
        self.response_char = response_char

    def callback(self, handle, value):
        value = value.decode("utf-8")
        self.manager.add_log_request(value, self.response_callback)

    def response_callback(self, response):
        self.response_char.respond(response)


class CabotLogResponseChar(BLENotifyChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, None)
        self.uuid = uuid

    def respond(self, response):
        jsonText = json.dumps(response, separators=(',', ':'))
        self.send_text(self.uuid, jsonText)


class DeviceStatus:
    def __init__(self):
        self.level = "Unknown"
        self.devices = []

    def ok(self):
        self.level = "OK"

    def error(self):
        self.level = "Error"

    def set_json(self, text):
        try:
            data=json.loads(text)
            self.devices = []
            if 'devices' in data:
                for dev in data['devices']:
                    device = {
                        'type': dev['device_type'],
                        'model': dev['device_model'],
                        'level': "OK" if dev['device_status'] == "0" else "Error",
                        'message': dev['device_message'],
                        'values': []
                    }
                    for key in dev:
                        device['values'].append({
                            'key': key,
                            'value': dev[key]
                        })
                    self.devices.append(device)
        except:
            logger.info(traceback.format_exc())

    @property
    def json(self):
        return {
            'level': self.level,
            'devices': self.devices
        }

class SystemStatus:
    def __init__(self):
        self.level = "Unknown"
        self.diagnostics = []

    def activating(self):
        self.level = "Activating"

    def deactivating(self):
        self.level = "Deactivating"

    def active(self):
        self.level = "Active"

    def inactive(self):
        self.level = "Inactive"

    def error(self):
        self.level = "Error"

    def set_diagnostics(self, diagnostics):
        self.diagnostics = diagnostics

    def is_active(self):
        if self.level == "Active":
            return True
        if len(self.diagnostics) > 0:
            return True
        return False

    @property
    def json(self):
        return {
            'level': self.level,
            'diagnostics': self.diagnostics
        }
     
def main (args=None):
    rclpy.init(args=args)
    
    node = CaBotNode_Pub()
    
    try:
        node.cabot_pub_event(msg)
    except:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ =='__main__':
    main()
