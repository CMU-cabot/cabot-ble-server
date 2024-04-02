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

import roslibpy
from roslibpy.comm import RosBridgeClientFactory

from cabot import util
from cabot.event import BaseEvent
from cabot_ui.event import NavigationEvent
from cabot_ace import BatteryDriverDelegate
from cabot_log_report import LogReport

CABOT_BLE_VERSION = "20230222"
rosbridge_host = os.environ.get('CABOT_ROSBRIDGE_HOST', 'localhost')


ble_manager = None

# settings for roslibpy reconnection
RosBridgeClientFactory.set_initial_delay(1)
RosBridgeClientFactory.set_max_delay(3)
client = roslibpy.Ros(host=rosbridge_host, port=9091)
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

diagnostics_topic = roslibpy.Topic(client, "/diagnostics_agg", "diagnostic_msgs/DiagnosticArray")
cabot_event_topic_sub = roslibpy.Topic(client, '/cabot/event', 'std_msgs/String')
cabot_event_topic_pub = roslibpy.Topic(client, '/cabot/event', 'std_msgs/String')
ble_hb_topic = roslibpy.Topic(client, '/cabot/ble_heart_beat', 'std_msgs/String')
activity_log_topic = roslibpy.Topic(client, '/cabot/activity_log', 'cabot_msgs/Log')
speak_service = roslibpy.Service(client, '/speak', 'cabot_msgs/Speak')

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

def activity_log(category="", text="", memo=""):
    now = roslibpy.Time.now()
    logger.info("category={}, text={}, memo={}".format(category, text, memo))
    try:
        activity_log_topic.publish(roslibpy.Message({
            'header': {
                'stamp': {
                    'secs': now.secs,
                    'nsecs': now.nsecs
                }
            },
            'category': category,
            'text': text,
            'memo': memo
        }))
    except:
        logger.info(traceback.format_exc())


diagnostics = []
def diagnostic_agg_callback(msg):
    global diagnostics
    diagnostics = msg['status']
    for diagnostic in diagnostics:
        # reduce floating number digits
        for i in range(len(diagnostic['values'])-1, -1, -1):
            value = diagnostic['values'][i]
            if value['key'] == 'Minimum acceptable frequency (Hz)' or \
               value['key'] == 'Maximum acceptable frequency (Hz)' or \
               value['key'] == 'Events in window' or \
               value['key'] == 'Events since startup':
                diagnostic['values'].pop(i)
                continue
            try:
                value['value'] = "%.2f"%(float(value['value']))
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
    activity_log("cabot/event", msg['data'])


cabot_event_topic_sub.subscribe(cabot_event_callback)
diagnostics_topic.subscribe(diagnostic_agg_callback)

@util.setInterval(1.0)
def polling_ros():
    global client
    if not client.is_connected:
        if ROS_CLIENT_CONNECTED[0]:
            logger.info("ROS bridge has been disconnected")
            ROS_CLIENT_CONNECTED[0] = False
            return

        logger.debug("polling")
        try:
            client.run(1.0)
            logger.info("ROS bridge is connected")
            ROS_CLIENT_CONNECTED[0] = True
        except Exception as e:
            # except Failed to connect to ROS
            pass
    else:
        pass

polling_ros()


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
            cabot_event_topic_pub.publish(roslibpy.Message({'data': str(event)}))

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
            cabot_event_topic_pub.publish(roslibpy.Message({'data': str(event)}))
            return

        logger.info("destination: %s", value)
        event = NavigationEvent(subtype="destination", param=value)
        cabot_event_topic_pub.publish(roslibpy.Message({'data': str(event)}))


class SummonsChar(BLESubChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def callback(self, handle, value):
        value = value.decode("utf-8")
        logger.info("summons_callback %s", value)
        event = NavigationEvent(subtype="summons", param=value)
        cabot_event_topic_pub.publish(roslibpy.Message({'data': str(event)}))


class HeartbeatChar(BLESubChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def callback(self, handle, value):
        value = value.decode("utf-8")
        logger.info("heartbeat(%s):%s", self.owner.address, value)
        ble_hb_topic.publish(roslibpy.Message({'data': value}))
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
