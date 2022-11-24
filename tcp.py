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

import gzip
import math
import queue
import os
import time
import json
import threading
import traceback
import logging
import re
import signal
import subprocess
import sys
from uuid import UUID
import eventlet
import socketio

import common

import roslibpy

from cabot import util
from cabot.event import BaseEvent
from cabot_ui.event import NavigationEvent
from cabot_ace import BatteryDriverNode, BatteryDriver, BatteryDriverDelegate, BatteryStatus

class CaBotTCP():

    def __init__(self, device, cabot_manager):
        self.target = device
        self.address = device.address
        self.sio = socketio.Server()
        self.app = socketio.WSGIApp(self.sio)
        self.cabot_manager = cabot_manager
        class _handler(socketio.Namespace):
            def __init__(self, cabot_manager):
                super().__init__("/cabot")
                self.manage_cabot_char = common.CabotManageChar(self, "mange_cabot", cabot_manager)
                self.log_char = common.CabotLogChar(self, "log", cabot_manager)
                self.summons_char = common.SummonsChar(self, "summons")
                self.destination_char = common.DestinationChar(self, "destination")
                self.heartbeat_char = common.HeartbeatChar(self, "heartbeat")

            @self.sio.event
            def manage_cabot(self, sid, data):
                self.manage_cabot_char.callback(0, data.encode("utf-8"))
            
            @self.sio.event
            def log(self, sid, data):
                self.log_char.callback(0, data.encode("utf-8"))

            @self.sio.event
            def summons(self, sid, data):
                self.summons_char.callback(0, data.encode("utf-8"))
            
            @self.sio.event
            def destination(self, sid, data):
                self.destination_char.callback(0, data.encode("utf-8"))

            @self.sio.event
            def heartbeat(self, sid, data):
                self.heartbeat_char.callback(0, data.encode("utf-8"))

        self.version_char = common.VersionChar(self, "version")

        self.device_status_char = common.StatusChar(self, "device_status", cabot_manager.device_status, interval=5)
        self.ros_status_char = common.StatusChar(self, "ros_status", cabot_manager.cabot_system_status, interval=5)
        self.battery_status_char = common.StatusChar(self, "battery_status", cabot_manager.cabot_battery_status, interval=5)

        self.speak_char = common.SpeakChar(self, "speak")
        self.event_char = common.EventChars(self, navi_uuid="navigate",
                                     content_uuid = "content",
                                     sound_uuid = "sound")
        self.handler = CaBotTCP._hander(self.cabot_manager)

        self.last_heartbeat = time.time()

        # speak
        self.alive = False
        self.ready = False

        self.error_count = 0


    def send_text(self, uuid, text, priority=10):
        self.sio.emit(uuid, text)

    def start(self):
        common.logger.info("CaBotTCP thread started")
        self.alive = True
        start_time = time.time()
        try:
            # if the device is disconnected and it already past 10 minutes then start over from scanning BLE MAC address
            # because iOS change MAC address pediodically
            while time.time() - start_time < 60*10 and self.alive:

                self.sio.register_namespace(self.handler)
                eventlet.wsgi.server(eventlet.listen(('', 5000)), self.app)

                error=False

                self.device_status_char.start()
                self.ros_status_char.start()
                self.battery_status_char.start()
                if error:
                    common.logger.error("cannot find characteristic")
                    break
                self.ready = True
                self.version_char.notify()

                # wait while heart beat is valid
                self.last_heartbeat = time.time()


            common.logger.error("exit while loop")
        except:
            common.logger.error(traceback.format_exc())
        finally:
            common.logger.error("stopping")
            for char in self.chars:
                if char.target:
                    char.target.stop()
            self.stop()
            common.logger.error("sending on_terminate")
            self.ble_manager.on_terminate(self)
        common.logger.info("CaBotBLE thread ended")

    def req_stop(self):
        self.alive = False

    def stop(self):
        self.alive = False
        self.ready = False
        self.device_status_char.stop()
        self.ros_status_char.stop()
        self.battery_status_char.stop()
        self.check_queue_stop.set()
        if self.target is not None:
            try:
                self.target.disconnect()
            except:
                common.logger.error(traceback.format_exc())


class BLEDeviceManager(dgatt.DeviceManager, object):
    def __init__(self, adapter_name, cabot_name=None, cabot_manager=None):
        super().__init__(adapter_name = adapter_name)
        self.cabot_name = "CaBot" + ("-" + cabot_name if cabot_name is not None else "")
        self.cabot_manager=cabot_manager
        common.logger.info("cabot_name: %s", self.cabot_name)
        self.bles_lock = threading.Lock()
        self.bles = {}
        self.speak_service = roslibpy.Service(common.client, '/speak', 'cabot_msgs/Speak')
        self.speak_service.advertise(self.handleSpeak)

    def handleSpeak(self, req, res):
        common.logger.info("/speak request (%s)", str(req))
        with self.bles_lock:
            for ble in self.bles.values():
                if ble.speak_char:
                    ble.speak_char.handleSpeak(req=req)
        res['result'] = True
        return True

    def handleEventCallback(self, msg):
        with self.bles_lock:
            for ble in self.bles.values():
                if ble.event_char:
                    ble.event_char.handleEventCallback(msg)

    def on_terminate(self, bledev):
        common.logger.info("terminate %s", bledev.target.path)
        with self.bles_lock:
            self.bles.pop(bledev.target.path)
            if len(self.bles) == 0:
                self.start_discovery(DISCOVERY_UUIDS)

    #def make_device(self, mac_address):
    #    return gatt.Device(mac_address=mac_address, manager=self)

    def device_discovered(self, device):
        if len(self.bles) == 0:
            common.logger.info("device {} {} discovered. bles.size={}".format(device.name, device.path, len(self.bles)))
        with self.bles_lock:
            if device.name == self.cabot_name:
                if not device.path in self.bles.keys():
                    common.logger.info("device {} {} discovered".format(device.name, device.path))
                    ble = CaBotBLE(device=device, ble_manager=self, cabot_manager=self.cabot_manager)
                    self.bles[device.path] = ble
                    ble.thread = threading.Thread(target=ble.start)
                    ble.thread.start()
                    self.stop_discovery()
                else:
                    #common.logger.info("device {} {} is already registered".format(device.alias, device.mac_address))
                    pass

    def stop(self):
        super().stop()
        with self.bles_lock:
            bles = list(self.bles.values())
            for ble in bles:
                ble.req_stop()
                ble.thread.join()
