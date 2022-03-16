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
import subprocess
from uuid import UUID

import pygatt
import gatt

import roslibpy

from cabot import util
from cabot.event import BaseEvent
from cabot_ui.event import NavigationEvent

CABOT_BLE_UUID = lambda _id: UUID("35CE{0:04X}-5E89-4C0D-A3F6-8A6A507C1BF1".format(_id))
CABOT_BLE_VERSION = "20220314"
DEBUG=False

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

client = roslibpy.Ros(host='localhost', port=9091)
ROS_CLIENT_CONNECTED = [False]

subscriber = roslibpy.Topic(client, "/diagnostics_agg", "diagnostic_msgs/DiagnosticArray")

@util.setInterval(1.0)
def polling_ros():
    if not client.is_connected:
        if ROS_CLIENT_CONNECTED[0]:
            logger.info("ROS bridge has been disconnected")
            ROS_CLIENT_CONNECTED[0] = False

        logger.debug("polling")
        try:
            client.run(1.0)
            logger.info("ROS bridge is connected")
            ROS_CLIENT_CONNECTED[0] = True
        except Exception:
            # except Failed to connect to ROS
            pass
    else:
        POLLING_STOP.set()
        logger.info("subscribe to diagnostic_agg")
        subscriber.subscribe(diagnostic_agg_callback)


POLLING_STOP=polling_ros()

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


class Diagnostics:
    def __init__(self, name, states):
        self.name = name.replace("/","")
        self.states = states
        for state in self.states:
            if state['name'] == name:
                self.states.remove(state)
                self.main_state = state

    @property
    def level(self):
        return self.main_state['level']

    @property
    def status(self):
        return {
            "name": self.name,
            "status": (self.level == 0),
            "message": self.messages
        }

    @property
    def messages(self):
        res = ""
        for state in self.states:
            name=state['name'].replace("/%s/"%(self.name), "")
            res += "%s: %s\n"%(name,state['message'])
            for entry in state['values']:
                res += "  %s:\n    %s\n"%(entry['key'], entry['value'])
        return res

    def __str__(self):
        res = "[%d]%s\n"%(self.status, self.name)
        res += self.messages
        return res

    def from_message(msg):
        top_level_names = []
        top_level_pat = re.compile("^/[^/]+$")
        for state in msg['status']:
            if top_level_pat.match(state['name']):
                top_level_names.append(state['name'])
        diagnostics = []
        for name in top_level_names:
            name_pat = re.compile("^%s.*"%(name))
            states = []
            for state in msg['status']:
                if name_pat.match(state['name']):
                    states.append(state)
            diagnostics.append(Diagnostics(name, states))
        return diagnostics

diagnostics = None
def diagnostic_agg_callback(msg):
    global diagnostics
    diagnostics = Diagnostics.from_message(msg)

class BLESubChar:
    def __init__(self, owner, uuid, indication=False):
        self.owner = owner
        self.uuid = uuid
        self.indication = indication
        self.valid = False

    def callback(self, handle, value):
        raise RuntimeError("callback is not implemented")

    def not_found(self):
        pass

    def subscribe_to(self, target):
        try:
            target.subscribe(self.uuid, self.callback, indication=self.indication)
            self.valid = True
        except pygatt.exceptions.BLEError:
            logger.info("could not connect to char %s", self.uuid)


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
            self.owner.event_topic.publish(roslibpy.Message({'data': str(event)}))
            return

        logger.info("destination: %s", value)
        event = NavigationEvent(subtype="destination", param=value)
        self.owner.event_topic.publish(roslibpy.Message({'data': str(event)}))


class SummonsChar(BLESubChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def callback(self, handle, value):
        value = value.decode("utf-8")
        logger.info("summons_callback %s", value)
        event = NavigationEvent(subtype="summons", param=value)
        self.owner.event_topic.publish(roslibpy.Message({'data': str(event)}))


class HeartbeatChar(BLESubChar):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def callback(self, handle, value):
        value = value.decode("utf-8")
        # logger.info("heartbeat(%s):%s", self.owner.address, value)
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

    def send_text(self, uuid, text):
        self.owner.send_text(uuid, text)


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
        self.loopStop = self._loop()
        self.count = 0

    @util.setInterval(1)
    def _loop(self):
        self.count += 1
        if self.interval <= self.count:
            self.count = 0
            self.notify()

    def notify(self):
        status = json.dumps(self.func(), separators=(',', ':'))
        self.send_text(self.uuid, status)

    def stop(self):
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

        self.send_text(self.uuid, text)
        return True

class EventChars:
    def __init__(self):
        self.event_topic = roslibpy.Topic(client, '/cabot/event', 'std_msgs/String')
        self.event_topic.subscribe(self._event_callback)

        self.navi_uuid = CABOT_BLE_UUID(0x300)
        self.content_uuid = CABOT_BLE_UUID(0x400)
        self.sound_uuid = CABOT_BLE_UUID(0x500)

    def _event_callback(self, msg):
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

    def __init__(self, address, ble_manager, cabot_manager):
        self.address = address
        self.ble_manager = ble_manager
        self.cabot_manager = cabot_manager
        self.chars = []

        self.version_char = VersionChar(self, CABOT_BLE_UUID(0x00))

        self.chars.append(CabotManageChar(self, CABOT_BLE_UUID(0x05), self.cabot_manager))
        self.device_status_char = StatusChar(self, CABOT_BLE_UUID(0x06), cabot_manager.device_status, interval=5)
        self.ros_status_char = StatusChar(self, CABOT_BLE_UUID(0x07), cabot_manager.cabot_system_status, interval=5)

        self.chars.append(SummonsChar(self, CABOT_BLE_UUID(0x09)))
        self.chars.append(DestinationChar(self, CABOT_BLE_UUID(0x10)))
        self.chars.append(HeartbeatChar(self, CABOT_BLE_UUID(0x9999)))

        self.speak_char = SpeakChar(self, CABOT_BLE_UUID(0x200))
        self.event_char = EventChars()

        self.adapter = pygatt.GATTToolBackend(max_read=2048)
        self.target = None
        self.last_heartbeat = time.time()

        # speak
        self.alive = False
        self.ready = False

        self.queue = queue.Queue()
        self.check_queue_stop = self.check_queue()

    def send_text(self, uuid, text):
        data = ("%s"%(text)).encode("utf-8")
        if not self.ready:
            logger.info("call async %s with data(%d), but not ready", uuid, len(data))
            return
        try:
            handle = self.target.get_handle(uuid)
        except:
            logger.info("Could not get handle")
            return
        self.queue.put((handle, data))

    def make_packets(self, data, size):
        length0 = len(data)
        data = bytearray(gzip.compress(data))
        length1 = len(data)
        packet_size = size - 4
        packets = []
        n = math.ceil(length1/packet_size)
        for i in range(0,n):
            temp = bytearray(data[i*packet_size:(i+1)*packet_size])
            temp[0:0] = length1.to_bytes(2,"big")
            temp[2:2] = (i*packet_size).to_bytes(2,"big")
            logger.info("packet[%d] = %d"%(i, len(temp)))
            packets.append(temp)
        logger.info("data/gzip length = %d/%d (%.0f%%)", length1, length0, length1/length0*100.0)
        return packets

    @util.setInterval(0.01)
    def check_queue(self):
        if self.queue.empty():
            return
        (handle, data) = self.queue.get()
        start = time.time()
        try:
            total = 0
            for packet in self.make_packets(data, 512):
                total += len(packet)
                self.target.char_write_handle(handle, value=packet, wait_for_response=True, timeout=2)
            logger.info("char_write %d bytes in %f seconds (%.2fKbps)",
                        total, (time.time()-start), total*8/(time.time()-start)/1024)
        except:
            logger.info(traceback.format_exc())

    def start(self):
        self.alive = True
        start_time = time.time()
        try:
            # if the device is disconnected and it already past 10 minutes then start over from scanning BLE MAC address
            # because iOS change MAC address pediodically
            while time.time() - start_time < 60*10 and self.alive:
                self.adapter.start(reset_on_start=False)
                self.target = None

                try:
                    logger.info("trying to connect to %s", self.address)
                    self.target = self.adapter.connect(self.address, timeout=15, address_type=pygatt.BLEAddressType.random)
                    self.target.exchange_mtu(64)
                except pygatt.exceptions.NotConnectedError:
                    logger.error("device not connected %s", self.address)
                    break
                except pygatt.exceptions.NotificationTimeout:
                    logger.error("setting exchange_mtu failed %s", self.address)
                    self.target = None
                    break

                # discover characteristics once to reduce waiting time if characteristics is not provided by the target
                target_chars = self.target.discover_characteristics()
                error=False
                for char in self.chars:
                    if target_chars.get(char.uuid):
                        char.subscribe_to(self.target)
                    else:
                        char.not_found()
                        error=True
                if error:
                    break
                self.ready = True
                self.version_char.notify()

                # wait while heart beat is valid
                self.last_heartbeat = time.time()
                timeout = 3.0
                while time.time() - self.last_heartbeat < timeout and self.alive:
                    if time.time() - self.last_heartbeat > timeout/2.0:
                        logger.info("No heartbeat, reconnecting in %.1f seconds %s", timeout - (time.time() - self.last_heartbeat), self.address)
                    time.sleep(0.5)
                self.ready = False

        except pygatt.exceptions.BLEError:
            logger.info("device disconnected")
        except:
            logger.error(traceback.format_exc())
        finally:
            self.stop()
            self.ble_manager.on_terminate(self)

    def req_stop(self):
        self.alive = False

    def stop(self):
        self.alive = False
        self.ready = False
        self.device_status_char.stop()
        self.ros_status_char.stop()
        self.check_queue_stop.set()
        if self.target is not None:
            try:
                self.target.disconnect()
            except pygatt.exceptions.BLEError:
                #device is already closed #logger.info("device disconnected")
                pass
        self.adapter.stop()


class BLEDeviceManager(gatt.DeviceManager, object):
    def __init__(self, adapter_name, cabot_name=None, cabot_manager=None):
        super().__init__(adapter_name = adapter_name)
        self.cabot_name = "CaBot" + ("-" + cabot_name if cabot_name is not None else "")
        self.cabot_manager=cabot_manager
        logger.info("cabot_name: %s", self.cabot_name)
        self.bles = {}
        self.service = roslibpy.Service(client, '/speak', 'cabot_msgs/Speak')
        self.service.advertise(self.handleSpeak)

    def handleSpeak(self, req, res):
        logger.info("/speak request (%s)", str(req))
        for ble in self.bles.values():
            if ble.speak_char:
                ble.speak_char.handleSpeak(req=req)
        res['result'] = True
        return True

    def on_terminate(self, bledev):
        logger.info("terminate %s", bledev.address)
        self.bles.pop(bledev.address)

    def make_device(self, mac_address):
        return gatt.Device(mac_address=mac_address, manager=self)

    def device_discovered(self, device):
        if device.alias() == self.cabot_name:
            if not device.mac_address in self.bles.keys():
                ble = CaBotBLE(address=device.mac_address, ble_manager=self, cabot_manager=self.cabot_manager)
                self.bles[device.mac_address] = ble
                thread = threading.Thread(target=ble.start)
                thread.start()

    def stop(self):
        for ble in self.bles.values():
            ble.req_stop()


class CaBotManager:
    def __init__(self):
        self.device_ok = False
        self._device_status = {}
        self.cabot_service_active = False
        self._cabot_system_status = {}
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
        
    @util.setInterval(5)
    def _run(self):
        self._run_once()

    def _run_once(self):
        self.run_count += 1
        if self.check_interval <= self.run_count:
            self._check_device_status()
            self._check_service_active()
            self.run_count = 0
            if self.device_ok and self.cabot_service_active:
                self.check_interval = min(self.check_interval+1, 1)
            else:
                self.check_interval = 1

        # logger.info("CaBotManager run %d %d %d %d %d",
        #             self.start_flag, self.device_ok, self.cabot_service_active, self.run_count, self.check_interval)
        if self.start_flag:
            if self.device_ok:
                self.start_flag = False
                if not self.cabot_service_active:
                    self.start()

    def _check_device_status(self):
        # ToDo: call check_device_status
        result = subprocess.run(["sudo", "docker-compose", "run", "--rm",  "check"], capture_output=True, text=True, cwd="/opt/cabot-device-check")
        # logger.info(result.returncode)
        # logger.info(result.stdout)
        if result.returncode == 0:
            self.device_ok = True
            self._device_status = {
                "devices": [
                    { "name": "Summary", "status": self.device_ok, "message": "Device check is OK" }
                ]
            }
        else:
            self.device_ok = False
            self._device_status = {
                "devices": [
                    { "name": "Summary", "status": self.device_ok, "message": "Something is wrong" }
                ]
            }

    def _check_service_active(self):
        if self._call(["systemctl", "--user", "--quiet", "is-active", "cabot"]) == 0:
            self.cabot_service_active = True
        else:
            self.cabot_service_active = False
        if diagnostics:
            self._cabot_system_status = {
                "is_active": self.cabot_service_active,
                "components": [d.status for d in diagnostics]
            }
        else:
            self._cabot_system_status = {
                "is_active": self.cabot_service_active,
                "components": []
            }

    def _call(self, command, lock=None):
        if lock is not None and not lock.acquire(blocking=False):
            logger.info("lock could not be acquired")
            return
        returncode = 1
        try:
            # logger.info("calling %s", str(command))
            returncode = subprocess.call(command)
        except:
            logger.error(traceback.format_exc())
        finally:
            if lock is not None:
                lock.release()
        return returncode

    def reboot(self):
        self._call(["sudo", "systemctl", "reboot"], lock=self.systemctl_lock)

    def poweroff(self):
        self._call(["sudo", "systemctl", "poweroff"], lock=self.systemctl_lock)

    def start(self):
        self._call(["systemctl", "--user", "start", "cabot"], lock=self.systemctl_lock)

    def stop(self):
        self._call(["systemctl", "--user", "stop", "cabot"], lock=self.systemctl_lock)

    def device_status(self):
        return self._device_status

    def cabot_system_status(self):
        return self._cabot_system_status


def main():
    cabot_name = os.environ['CABOT_NAME'] if 'CABOT_NAME' in os.environ else None
    adapter_name = os.environ['CABOT_BLE_ADAPTOR'] if 'CABOT_BLE_ADAPTOR' in os.environ else "hci0"
    start_at_launch = (os.environ['CABOT_START_AT_LAUNCH'] == "1") if 'CABOT_START_AT_LAUNCH' in os.environ else False

    cabot_manager = CaBotManager()
    cabot_manager.run(start=start_at_launch)

    ble_manager = BLEDeviceManager(adapter_name=adapter_name, cabot_name=cabot_name, cabot_manager=cabot_manager)

    # power on the adapter
    if not ble_manager.is_adapter_powered:
        ble_manager.is_adapter_powered = True

    ble_manager.start_discovery(["35CE0000-5E89-4C0D-A3F6-8A6A507C1BF1"])
    #ble_manager.start_discovery()

    try:
        ble_manager.run()
    except:
        logger.info(traceback.format_exc())
    finally:
        ble_manager.stop()
        ble_manager._main_loop.quit()
        client.terminate()

if __name__ == "__main__":
    main()
