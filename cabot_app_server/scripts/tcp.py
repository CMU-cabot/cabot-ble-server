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
from flask import Flask
import socketio

import common

import roslibpy

from cabot_common import util
from cabot_common.event import BaseEvent
from cabot_ui.event import NavigationEvent
from cabot_ace import BatteryDriverNode, BatteryDriver, BatteryDriverDelegate, BatteryStatus

class CaBotTCP():

    def __init__(self, cabot_manager):
        self.sio = socketio.Server(async_mode="threading", cors_allowed_origins="*")
        self.app = Flask(__name__)
        self.address = "tcp"
        self.cabot_manager = cabot_manager
        self.manage_cabot_char = common.CabotManageChar(self, "manage_cabot", cabot_manager)
        self.log_request_char = common.CabotLogRequestChar(self, "log_request",
                                                            cabot_manager, 
                                                            common.CabotLogResponseChar(self, "log_response"))
        self.log_char = common.CabotLogChar(self, "log", cabot_manager)
        self.summons_char = common.SummonsChar(self, "summons")
        self.destination_char = common.DestinationChar(self, "destination")
        self.heartbeat_char = common.HeartbeatChar(self, "heartbeat")
        class subchar_handler(socketio.Namespace):
            @self.sio.event
            def manage_cabot(sid, data):
                self.manage_cabot_char.callback(0, data[0].encode("utf-8"))
            
            @self.sio.event
            def log(sid, data):
                self.log_char.callback(0, data[0].encode("utf-8"))

            @self.sio.event
            def summons(sid, data):
                self.summons_char.callback(0, data[0].encode("utf-8"))
            
            @self.sio.event
            def destination(sid, data):
                self.destination_char.callback(0, data[0].encode("utf-8"))

            @self.sio.event
            def heartbeat(sid, data):
                self.heartbeat_char.callback(0, data[0].encode("utf-8"))

            @self.sio.event
            def req_version(sid, data):
                self.version_char.notify()

            @self.sio.event
            def connect(sid, environ, auth):
                common.logger.info("new socket.io connection")
                #self.version_char.notify()

            @self.sio.event
            def log_request(sid, data):
                self.log_request_char.callback(0, data[0])

            @self.sio.event
            def share(sid, data):
                common.logger.info(f"share {data[0]}")
                self.sio.emit("share", data[0])


        self.version_char = common.VersionChar(self, "cabot_version")

        self.device_status_char = common.StatusChar(self, "device_status", cabot_manager.device_status, interval=5)
        self.system_status_char = common.StatusChar(self, "system_status", cabot_manager.cabot_system_status, interval=5)
        self.battery_status_char = common.StatusChar(self, "battery_status", cabot_manager.cabot_battery_status, interval=5)

        self.speak_char = common.SpeakChar(self, "speak")
        self.event_char = common.EventChars(self, "navigate")
        self.touch_char = common.TouchChars(self, "touch")

        self.handler = subchar_handler("/cabot")
        self.sio.register_namespace(self.handler)
        self.app.wsgi_app = socketio.WSGIApp(self.sio, wsgi_app=self.app.wsgi_app)

        self.last_heartbeat = time.time()

        # speak
        self.alive = False
        self.ready = False

        self.error_count = 0

    def send_text(self, uuid, text, priority=10):
        self.sio.emit(uuid, text)

    def handleSpeak(self, req, res):
        common.logger.info("/speak request tcp (%s)", str(req))
        self.speak_char.handleSpeak(req=req)
        res.result = True
        common.logger.info("/speak request tcp end")
        return True

    def handleEventCallback(self, msg, request_id):
        self.event_char.handleEventCallback(msg, request_id)

    def handleTouchCallback(self, msg):
        self.touch_char.handleTouchCallback(msg)

    def start(self):
        common.logger.info("CaBotTCP thread started")
        self.alive = True
        start_time = time.time()
        try:
            self.device_status_char.start()
            self.system_status_char.start()
            self.battery_status_char.start()
            self.ready = True

            # wait while heart beat is valid
            self.last_heartbeat = time.time()
            common.logger.info("CaBotTCP listening...")
            self.app.run(host='0.0.0.0', port=5000)

        except(KeyboardInterrupt, SystemExit):
            common.logger.info("stopping tcp server...")
            self.stop()
        except:
            common.logger.error(traceback.format_exc())


    def stop(self):
        common.logger.info("CaBotTCP thread stop")
        self.alive = False
        self.ready = False
        self.device_status_char.stop()
        self.system_status_char.stop()
        self.battery_status_char.stop()
        #if self.wsgisrv is not None:
        #    self.wsgisrv.stop()
        #    self.wsgisrv.close()


