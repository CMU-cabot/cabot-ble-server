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

import roslibpy
from roslibpy.comm import RosBridgeClientFactory

from cabot import util
from cabot.event import BaseEvent
from cabot_ui.event import NavigationEvent
from cabot_ace import BatteryDriverNode, BatteryDriver, BatteryDriverDelegate, BatteryStatus

CABOT_BLE_UUID = lambda _id: UUID("35CE{0:04X}-5E89-4C0D-A3F6-8A6A507C1BF1".format(_id))
CABOT_BLE_VERSION = "20220320"
MTU_SIZE = 2**10 # could be 2**15, but 2**10 is enough
CHAR_WRITE_MAX_SIZE = 512 # should not be exceeded this value
#DISCOVERY_UUIDS=[str(CABOT_BLE_VERSION(0))]
DISCOVERY_UUIDS=[]
WAIT_AFTER_CONNECTION=0.25 # wait a bit after connection to avoid error
DEBUG=False

ble_manager = None

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

# settings for roslibpy reconnection
RosBridgeClientFactory.set_initial_delay(1)
RosBridgeClientFactory.set_max_delay(3)
client = roslibpy.Ros(host='localhost', port=9091)
ROS_CLIENT_CONNECTED = [False]

diagnostics_topic = roslibpy.Topic(client, "/diagnostics_agg", "diagnostic_msgs/DiagnosticArray")
event_topic = roslibpy.Topic(client, '/cabot/event', 'std_msgs/String')
ble_hb_topic = roslibpy.Topic(client, '/cabot/ble_heart_beat', 'std_msgs/String')
activity_log_topic = roslibpy.Topic(client, '/cabot/activity_log', 'cabot_msgs/Log')

@util.setInterval(1.0)
def polling_ros():
    global client
    if not client.is_connected:
        if ROS_CLIENT_CONNECTED[0]:
            logger.info("ROS bridge has been disconnected")
            ROS_CLIENT_CONNECTED[0] = False

        logger.debug("polling")
        try:
            client.run(1.0)
            logger.info("ROS bridge is connected")
            logger.info("subscribe to diagnostic_agg")
            diagnostics_topic.subscribe(diagnostic_agg_callback)
            event_topic.subscribe(event_callback)

            ROS_CLIENT_CONNECTED[0] = True
        except Exception as e:
            # except Failed to connect to ROS
            pass
    else:
        pass

polling_ros()

### Debug
def set_debug_mode():
    from logging import StreamHandler, Formatter

    for key in logging.Logger.manager.loggerDict:
        #for key in ["pygatt.device"]:
        try:
            logging.Logger.manager.loggerDict[key].setLevel(logging.DEBUG)
        except:
            pass

if DEBUG:
    set_debug_mode()

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

def event_callback(msg):
    logger.info("event_callback is called")
    if ble_manager:
        activity_log("cabot/event", msg['data'])
        ble_manager.handleEventCallback(msg)
    else:
        logger.error("There is no ble_manager instance")

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
            self.manager.reboot()
        if value == "poweroff":
            self.manager.poweroff()
        if value == "stop":
            self.manager.stop()
        if value == "start":
            self.manager.start()

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
            event_topic.publish(roslibpy.Message({'data': str(event)}))
            return

        logger.info("destination: %s", value)
        event = NavigationEvent(subtype="destination", param=value)
        event_topic.publish(roslibpy.Message({'data': str(event)}))


class SummonsChar(BLESubChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def callback(self, handle, value):
        value = value.decode("utf-8")
        logger.info("summons_callback %s", value)
        event = NavigationEvent(subtype="summons", param=value)
        event_topic.publish(roslibpy.Message({'data': str(event)}))


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

    def notify(self):
        logger.info("sending version")
        self.send_text(self.uuid, CABOT_BLE_VERSION)


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
        if self.func():
            self.func().stop()
        if self.loopStop:
            self.loopStop.set()


class SpeakChar(BLENotifyChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def handleSpeak(self, req):
        if not self.owner.ready:
            return None
        text = req['text']
        force = req['force']
        if force:
            text = "__force_stop__\n" + text

        self.send_text(self.uuid, text, priority=0)
        activity_log("ble speech request", req['text'], str(req['force']))
        return True


class EventChars(BLENotifyChar):
    def __init__(self, owner, navi_uuid, content_uuid, sound_uuid):
        super().__init__(owner, None) # uuid is not set because EventChars uses multiple uuids.
        self.navi_uuid = navi_uuid
        self.content_uuid = content_uuid
        self.sound_uuid = sound_uuid

    def handleEventCallback(self, msg):
        event = BaseEvent.parse(msg['data'])
        if event is None:
            logger.error("cabot event %s cannot be parsed", msg['data'])
            return

        if event.type != NavigationEvent.TYPE:
            return

        if event.subtype == "next":
            # notify the phone next event
            self.send_text(self.navi_uuid, "next")

        if event.subtype == "arrived":
            self.send_text(self.navi_uuid, "arrived")

        if event.subtype == "content":
            self.send_text(self.content_uuid, event.param)

        if event.subtype == "sound":
            self.send_text(self.sound_uuid, event.param)

class CaBotBLE:

    def __init__(self, device, ble_manager, cabot_manager):
        self.target = device
        self.address = device.address
        self.ble_manager = ble_manager
        self.cabot_manager = cabot_manager
        self.chars = []

        self.version_char = VersionChar(self, CABOT_BLE_UUID(0x00))

        self.chars.append(CabotManageChar(self, CABOT_BLE_UUID(0x01), self.cabot_manager))
        self.device_status_char = StatusChar(self, CABOT_BLE_UUID(0x02), cabot_manager.device_status, interval=5)
        self.ros_status_char = StatusChar(self, CABOT_BLE_UUID(0x03), cabot_manager.cabot_system_status, interval=5)
        self.battery_status_char = StatusChar(self, CABOT_BLE_UUID(0x04), cabot_manager.cabot_battery_status, interval=5)
        self.chars.append(CabotLogChar(self, CABOT_BLE_UUID(0x05), self.cabot_manager))

        self.chars.append(SummonsChar(self, CABOT_BLE_UUID(0x10)))
        self.chars.append(DestinationChar(self, CABOT_BLE_UUID(0x11)))

        self.speak_char = SpeakChar(self, CABOT_BLE_UUID(0x30))
        self.event_char = EventChars(self, navi_uuid=CABOT_BLE_UUID(0x40),
                                     content_uuid = CABOT_BLE_UUID(0x50),
                                     sound_uuid = CABOT_BLE_UUID(0x60))

        self.chars.append(HeartbeatChar(self, CABOT_BLE_UUID(0x9999)))

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
            #logger.info("packet[%d] = %d"%(i, len(temp)))
            packets.append(temp)
        logger.info("data/gzip length = %d/%d (%.0f%%)", length1, length0, length1/length0*100.0)
        return packets

    @util.setInterval(0.01)
    def check_queue(self):
        if not self.ready:
            return
        if self.queue.empty():
            return
        logger.info("queue size = {}".format(self.queue.qsize()))
        (priority, uuid, data) = self.queue.get()
        try:
            handle = self.target.get_handle(uuid)
        except:
            logger.error(traceback.format_exc())
            logger.info("Could not get handle {}", )
            return

        start = time.time()
        try:
            total = 0
            for packet in self.make_packets(data, CHAR_WRITE_MAX_SIZE):
                total += len(packet)
                self.target.char_write_handle(handle, value=packet, wait_for_response=True, timeout=2)
            logger.info("char_write %d bytes in %f seconds (%.2fKbps)",
                        total, (time.time()-start), total*8/(time.time()-start)/1024)
            self.error_count = 0
        except:
            self.error_count += 1
            if self.error_count > 3:
                self.alive = False
            else:
                self.queue.put((priority, uuid, data))
                #logger.info(traceback.format_exc())
                logger.error("check_queue got an error {}".format(self.error_count))

    def start(self):
        logger.info("CaBotBLE thread started")
        self.alive = True
        start_time = time.time()
        try:
            # if the device is disconnected and it already past 10 minutes then start over from scanning BLE MAC address
            # because iOS change MAC address pediodically
            while time.time() - start_time < 60*10 and self.alive:
                try:
                    logger.info("trying to connect to %s", self.target)
                    if not self.target.connect():
                        logger.error("Cannot connect to {}".format(self.target))
                        break
                except:
                    logger.error(traceback.format_exc())
                    break

                if not self.target.connected:
                    logger.info("not connected")
                    self.alive = False
                    break
                logger.info("connected")
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
                    logger.error("cannot find characteristic")
                    break
                self.ready = True
                self.version_char.notify()

                # wait while heart beat is valid
                self.last_heartbeat = time.time()
                timeout = 3.0
                while time.time() - self.last_heartbeat < timeout and self.alive:
                    if time.time() - self.last_heartbeat > timeout/2.0:
                        logger.warning("No heartbeat, reconnecting in %.1f seconds %s", timeout - (time.time() - self.last_heartbeat), self.address)
                    time.sleep(0.5)
                self.ready = False
                self.alive = False

            logger.error("exit while loop")
        except:
            logger.error(traceback.format_exc())
        finally:
            logger.error("stopping")
            for char in self.chars:
                if char.target:
                    char.target.stop()
            self.stop()
            logger.error("sending on_terminate")
            self.ble_manager.on_terminate(self)
        logger.info("CaBotBLE thread ended")

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
                logger.error(traceback.format_exc())


class BLEDeviceManager(dgatt.DeviceManager, object):
    def __init__(self, adapter_name, cabot_name=None, cabot_manager=None):
        super().__init__(adapter_name = adapter_name)
        self.cabot_name = "CaBot" + ("-" + cabot_name if cabot_name is not None else "")
        self.cabot_manager=cabot_manager
        logger.info("cabot_name: %s", self.cabot_name)
        self.bles_lock = threading.Lock()
        self.bles = {}
        self.speak_service = roslibpy.Service(client, '/speak', 'cabot_msgs/Speak')
        self.speak_service.advertise(self.handleSpeak)

    def handleSpeak(self, req, res):
        logger.info("/speak request (%s)", str(req))
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
        logger.info("terminate %s", bledev.target.path)
        with self.bles_lock:
            self.bles.pop(bledev.target.path)
            if len(self.bles) == 0:
                self.start_discovery(DISCOVERY_UUIDS)

    #def make_device(self, mac_address):
    #    return gatt.Device(mac_address=mac_address, manager=self)

    def device_discovered(self, device):
        if len(self.bles) == 0:
            logger.info("device {} {} discovered. bles.size={}".format(device.name, device.path, len(self.bles)))
        with self.bles_lock:
            if device.name == self.cabot_name:
                if not device.path in self.bles.keys():
                    logger.info("device {} {} discovered".format(device.name, device.path))
                    ble = CaBotBLE(device=device, ble_manager=self, cabot_manager=self.cabot_manager)
                    self.bles[device.path] = ble
                    ble.thread = threading.Thread(target=ble.start)
                    ble.thread.start()
                    self.stop_discovery()
                else:
                    #logger.info("device {} {} is already registered".format(device.alias, device.mac_address))
                    pass

    def stop(self):
        super().stop()
        with self.bles_lock:
            bles = list(self.bles.values())
            for ble in bles:
                ble.req_stop()
                ble.thread.join()

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

    def stop(self):
        pass

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

    def stop(self):
        self.deactivating()
        self.diagnostics = []

class CaBotManager(BatteryDriverDelegate):
    def __init__(self):
        self._device_status = DeviceStatus()
        self._cabot_system_status = SystemStatus()
        self._battery_status = None
        self.systemctl_lock = threading.Lock()
        self.start_flag = False
        self.stop_run = None
        self.check_interval = 1
        self.run_count = 0

    def run(self, start=False):
        self.start_flag=start
        self._run_once()
        self.stop_run = self._run()

    def stop(self):
        if self.stop_run:
            self.stop_run.set()

    # BatteryDriverDelegate start
    def battery_status(self, status):
        self._battery_status = status
        if status.shutdown or status.lowpower_shutdown:
            logger.info("shutdown requested")
            self.stop()
            self.poweroff()
    # BatteryDriverDelegate end

    @util.setInterval(5)
    def _run(self):
        self._run_once()

    def _run_once(self):
        self.run_count += 1
        if self.check_interval <= self.run_count:
            self._check_device_status()
            self._check_service_active()
            self.run_count = 0

        if self.start_flag:
            if self._device_status.level == "OK":
                self.start_flag = False
                if self._cabot_system_status.level != "Active":
                    self.start()
            else:
                logger.info("Start at launch is requested, but device is not OK")

    def _check_device_status(self):
        if self._cabot_system_status.is_active():
            result = self._runprocess(["sudo", "-E", "./check_device_status.sh", "-j", "-s"])
        else:
            result = self._runprocess(["sudo", "-E", "./check_device_status.sh", "-j"])
        if result and result.returncode == 0:
            self._device_status.ok()
        else:
            self._device_status.error()
        self._device_status.set_json(result.stdout)

    def _check_service_active(self):
        result = self._runprocess(["systemctl", "--user", "is-active", "cabot"])
        if not result:
            return
        if result.returncode == 0:
            self._cabot_system_status.active()
        else:
            if result.stdout.strip() == "inactive":
                self._cabot_system_status.inactive()
            elif result.stdout.strip() == "failed":
                self._cabot_system_status.inactive()
            elif result.stdout.strip() == "deactivating":
                self._cabot_system_status.deactivating()
            else:
                logger.info("check_service_active unknown status: %s", result.stdout.strip())

        global diagnostics
        self._cabot_system_status.set_diagnostics(diagnostics)
        diagnostics = []

    def _runprocess(self, command):
        return subprocess.run(command, capture_output=True, text=True, env=os.environ.copy())

    def _call(self, command, lock=None):
        result = 0
        if lock is not None and not lock.acquire(blocking=False):
            logger.info("lock could not be acquired")
            return result
        try:
            # logger.info("calling %s", str(command))
            result = subprocess.call(command)
        except:
            logger.error(traceback.format_exc())
        finally:
            if lock is not None:
                lock.release()
        return result

    def reboot(self):
        self._call(["sudo", "systemctl", "reboot"], lock=self.systemctl_lock)

    def poweroff(self):
        self._call(["sudo", "systemctl", "poweroff"], lock=self.systemctl_lock)

    def start(self):
        self._call(["systemctl", "--user", "start", "cabot"], lock=self.systemctl_lock)
        self._cabot_system_status.activating()

    def stop(self):
        self._call(["systemctl", "--user", "stop", "cabot"], lock=self.systemctl_lock)
        self._cabot_system_status.deactivating()

    def device_status(self):
        return self._device_status

    def cabot_system_status(self):
        return self._cabot_system_status

    def cabot_battery_status(self):
        return self._battery_status

quit_flag=False
def sigint_handler(sig, frame):
    logger.info("sigint_handler")
    global quit_flag
    if sig == signal.SIGINT:
        ble_manager.stop()
        quit_flag=True
    else:
        logger.error("Unexpected signal")

def main():
    signal.signal(signal.SIGINT, sigint_handler)
    global ble_manager
    cabot_name = os.environ['CABOT_NAME'] if 'CABOT_NAME' in os.environ else None
    adapter_name = os.environ['CABOT_BLE_ADAPTER'] if 'CABOT_BLE_ADAPTER' in os.environ else "hci0"
    start_at_launch = (os.environ['CABOT_START_AT_LAUNCH'] == "1") if 'CABOT_START_AT_LAUNCH' in os.environ else False

    port_name = os.environ['CABOT_ACE_BATTERY_PORT'] if 'CABOT_ACE_BATTERY_PORT' in os.environ else None
    baud = int(os.environ['CABOT_ACE_BATTERY_BAUD']) if 'CABOT_ACE_BATTERY_BAUD' in os.environ else None


    cabot_manager = CaBotManager()
    cabot_manager.run(start=start_at_launch)

    driver = None
    if port_name is not None and baud is not None:
        driver = BatteryDriver(port_name, baud, delegate=cabot_manager)
        battery_driver_node = BatteryDriverNode(client, driver)
        battery_thread = threading.Thread(target=driver.start)
        battery_thread.start()

    result = subprocess.call(["grep", "-E", "^ControllerMode *= *le$", "/etc/bluetooth/main.conf"])
    if result != 0:
        logger.error("Please check your /etc/bluetooth/main.conf")
        line = subprocess.check_output(["grep", "-E", "ControllerMode", "/etc/bluetooth/main.conf"])
        logger.error("Your ControllerMode is '{}'".format(line.decode('utf-8').replace('\n', '')))
        logger.error("Please use ./setup_bluetooth_conf.sh to configure LE mode")
        sys.exit(result)

    try:
        while not quit_flag:
            result = subprocess.call(["sudo", "rfkill", "unblock", "bluetooth"])
            if result != 0:
                logger.error("Could not unblock rfkill bluetooth")
                time.sleep(1)
                continue

            result = subprocess.call(["sudo", "systemctl", "restart", "bluetooth"])
            if result != 0:
                logger.error("Could not restart bluetooth service")
                time.sleep(1)
                continue

            ble_manager = BLEDeviceManager(adapter_name=adapter_name, cabot_name=cabot_name, cabot_manager=cabot_manager)
            # power on the adapter
            try:
                while not ble_manager.is_adapter_powered and not quit_flag:
                    logger.info("Bluetooth is off, so powering on")
                    ble_manager.is_adapter_powered = True
                    time.sleep(1)
                ble_manager.start_discovery(DISCOVERY_UUIDS)
                ble_manager.run()
            except KeyboardInterrupt:
                logger.info("keyboard interrupt")
                break
            except:
                logger.info(traceback.format_exc())
                time.sleep(1)
    except KeyboardInterrupt:
        logger.info("keyboard interrupt")
    except Exception as e:
        logger.info(traceback.format_exc())
    finally:
        try:
            if driver:
                driver.stop()
                battery_thread.join()
            ble_manager.stop()
            client.terminate()
        except:
            logger.info(traceback.format_exc())

if __name__ == "__main__":
    main()
