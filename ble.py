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

import dgatt
import common

import roslibpy

from cabot import util
from cabot.event import BaseEvent
from cabot_ui.event import NavigationEvent
from cabot_ace import BatteryDriverNode, BatteryDriver, BatteryDriverDelegate, BatteryStatus

CABOT_BLE_UUID = lambda _id: UUID("35CE{0:04X}-5E89-4C0D-A3F6-8A6A507C1BF1".format(_id))
MTU_SIZE = 2**10 # could be 2**15, but 2**10 is enough
CHAR_WRITE_MAX_SIZE = 512 # should not be exceeded this value
DISCOVERY_UUIDS=[]
WAIT_AFTER_CONNECTION=0.25 # wait a bit after connection to avoid error

class CaBotBLE:

    def __init__(self, device, ble_manager, cabot_manager):
        self.target = device
        self.address = device.address
        self.ble_manager = ble_manager
        self.cabot_manager = cabot_manager
        self.chars = []

        self.version_char = common.VersionChar(self, CABOT_BLE_UUID(0x00))

        self.chars.append(common.CabotManageChar(self, CABOT_BLE_UUID(0x01), self.cabot_manager))
        self.device_status_char = common.StatusChar(self, CABOT_BLE_UUID(0x02), cabot_manager.device_status, interval=5)
        self.ros_status_char = common.StatusChar(self, CABOT_BLE_UUID(0x03), cabot_manager.cabot_system_status, interval=5)
        self.battery_status_char = common.StatusChar(self, CABOT_BLE_UUID(0x04), cabot_manager.cabot_battery_status, interval=5)
        self.chars.append(common.CabotLogChar(self, CABOT_BLE_UUID(0x05), self.cabot_manager))

        self.chars.append(common.SummonsChar(self, CABOT_BLE_UUID(0x10)))
        self.chars.append(common.DestinationChar(self, CABOT_BLE_UUID(0x11)))

        self.speak_char = common.SpeakChar(self, CABOT_BLE_UUID(0x30))
        self.event_char = common.EventChars(self, CABOT_BLE_UUID(0x40))

        self.chars.append(common.CabotLogRequestChar(self, CABOT_BLE_UUID(0x50), self.cabot_manager))
        self.log_response_char = common.CabotLogResponseChar(self, CABOT_BLE_UUID(0x51))

        self.chars.append(common.HeartbeatChar(self, CABOT_BLE_UUID(0x9999)))

        self.last_heartbeat = time.time()

        # speak
        self.alive = False
        self.ready = False

        self.queue = queue.PriorityQueue()
        self.check_queue_stop = self.check_queue()
        self.error_count = 0

    def send_text(self, uuid, text, priority=10):
        data = ("%s"%(text)).encode("utf-8")
        self.send_data(uuid, data, priority)

    def send_data(self, uuid, data, priority=10):
        if not self.ready:
            return
        self.queue.put((priority, uuid, data))

    def make_packets(self, orig_data, size):
        length0 = len(orig_data)
        data = bytearray(gzip.compress(orig_data))
        length1 = len(data)
        if length0 < length1:
            temp = bytearray(orig_data)
            temp[0:0] = length0.to_bytes(2,"big")
            temp[2:2] = int(0).to_bytes(2,"big")
            return [temp]
        packet_size = size - 4
        packets = []
        n = math.ceil(length1/packet_size)
        for i in range(0,n):
            temp = bytearray(data[i*packet_size:(i+1)*packet_size])
            temp[0:0] = length1.to_bytes(2,"big")
            temp[2:2] = (i*packet_size).to_bytes(2,"big")
            #common.logger.info("packet[%d] = %d"%(i, len(temp)))
            packets.append(temp)
        common.logger.info("data/gzip length = %d/%d (%.0f%%)", length1, length0, length1/length0*100.0)
        return packets

    @util.setInterval(0.01)
    def check_queue(self):
        if not self.ready:
            return
        if self.queue.empty():
            return
        common.logger.info("queue size = {}".format(self.queue.qsize()))
        (priority, uuid, data) = self.queue.get()
        try:
            handle = self.target.get_handle(uuid)
        except:
            common.logger.error(traceback.format_exc())
            common.logger.info("Could not get handle {}", )
            return

        start = time.time()
        try:
            total = 0
            for packet in self.make_packets(data, CHAR_WRITE_MAX_SIZE):
                total += len(packet)
                self.target.char_write_handle(handle, value=packet, wait_for_response=True, timeout=2)
            common.logger.info("char_write %d bytes in %f seconds (%.2fKbps)",
                        total, (time.time()-start), total*8/(time.time()-start)/1024)
            self.error_count = 0
        except:
            self.error_count += 1
            if self.error_count > 3:
                self.alive = False
            else:
                self.queue.put((priority, uuid, data))
                #common.logger.info(traceback.format_exc())
                common.logger.error("check_queue got an error {}".format(self.error_count))

    def start(self):
        common.logger.info("CaBotBLE thread started")
        self.alive = True
        start_time = time.time()
        try:
            # if the device is disconnected and it already past 10 minutes then start over from scanning BLE MAC address
            # because iOS change MAC address pediodically
            while time.time() - start_time < 60*10 and self.alive:
                try:
                    common.logger.info("trying to connect to %s", self.target)
                    if not self.target.connect():
                        common.logger.error("Cannot connect to {}".format(self.target))
                        break
                except:
                    common.logger.error(traceback.format_exc())
                    break

                if not self.target.connected:
                    common.logger.info("not connected")
                    self.alive = False
                    break
                common.logger.info("connected")
                time.sleep(WAIT_AFTER_CONNECTION)

                error=False
                for char in self.chars:
                    target = self.target.get_characteristic(char.uuid)
                    if target:
                        char.subscribe_to(target)
                    else:
                        char.not_found()
                        error=True
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
                timeout = 3.0
                while time.time() - self.last_heartbeat < timeout and self.alive:
                    if time.time() - self.last_heartbeat > timeout/2.0:
                        common.logger.warning("No heartbeat, reconnecting in %.1f seconds %s", timeout - (time.time() - self.last_heartbeat), self.address)
                    time.sleep(0.5)
                self.ready = False
                self.alive = False

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

    def handleSpeak(self, req, res):
        common.logger.info("/speak request ble (%s)", str(req))
        with self.bles_lock:
            for ble in self.bles.values():
                if ble.speak_char:
                    ble.speak_char.handleSpeak(req=req)
        res['result'] = True
        return True

    def handleEventCallback(self, msg, request_id):
        with self.bles_lock:
            for ble in self.bles.values():
                if ble.event_char:
                    ble.event_char.handleEventCallback(msg, request_id)

    def logResponse(self, response):
        common.logger.info("cabot log response")
        with self.bles_lock:
            for ble in self.bles.values():
                if ble.log_response_char:
                    ble.log_response_char.response(response)

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
