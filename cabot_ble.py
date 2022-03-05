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

import pygatt
import gatt

import roslibpy

from cabot import util
from cabot.event import BaseEvent
from cabot_ui.event import NavigationEvent

CABOT_BLE_UUID = lambda _id: UUID("35CE{0:04X}-5E89-4C0D-A3F6-8A6A507C1BF1".format(_id))
CABOT_BLE_VERSION = "1"
DEBUG=False

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

client = roslibpy.Ros(host='localhost', port=9091)
ROS_CLIENT_CONNECTED = [False]

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


class BLECharacteristic:
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


class VersionChar(BLECharacteristic):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def callback(self, handle, value):
        version = value.decode("utf-8")
        if version != CABOT_BLE_VERSION:
            logger.error("BLE Version mismatch %s != %s", CABOT_BLE_VERSION, value)
        else:
            logger.info("BLE Version matched %s", version)

    def not_found(self):
        logger.error("version number is not implemented")

class SystemctlChar(BLECharacteristic):
    def __init__(self, owner, uuid, command):
        super().__init__(owner, uuid)
        self.command = command

    def callback(self, handle, value):
        subprocess.call(self.command)

    def not_found(self):
        logger.error("%s is not implemented", " ".join(self.command))

class DestinationChar(BLECharacteristic):
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

class SummonsChar(BLECharacteristic):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def callback(self, handle, value):
        value = value.decode("utf-8")
        logger.info("summons_callback %s", value)
        event = NavigationEvent(subtype="summons", param=value)
        self.owner.event_topic.publish(roslibpy.Message({'data': str(event)}))

class HeartbeatChar(BLECharacteristic):
    def __init__(self, owner, uuid):
        super().__init__(owner, uuid)

    def callback(self, handle, value):
        value = value.decode("utf-8")
        logger.info("heartbeat(%s):%s", self.owner.address, value)
        self.owner.last_heartbeat = time.time()


class CaBotBLE:

    def __init__(self, address, manager):
        self.address = address
        self.manager = manager
        self.chars = []

        self.chars.append(VersionChar(self, CABOT_BLE_UUID(0x00)))
        self.chars.append(SystemctlChar(self, CABOT_BLE_UUID(0x1000), ["sudo", "systemctl", "reboot"]))
        self.chars.append(SystemctlChar(self, CABOT_BLE_UUID(0x1001), ["sudo", "systemctl", "poweroff"]))
        self.chars.append(SystemctlChar(self, CABOT_BLE_UUID(0x1002), ["systemctl", "--user", "restart", "cabot"]))
        self.chars.append(SummonsChar(self, CABOT_BLE_UUID(0x09)))
        self.chars.append(DestinationChar(self, CABOT_BLE_UUID(0x10)))
        self.chars.append(HeartbeatChar(self, CABOT_BLE_UUID(0x9999)))

        self.speak_uuid = CABOT_BLE_UUID(0x200)
        self.navi_uuid = CABOT_BLE_UUID(0x300)
        self.content_uuid = CABOT_BLE_UUID(0x400)
        self.sound_uuid = CABOT_BLE_UUID(0x500)
        self.data_buffer = {}

        self.event_topic = roslibpy.Topic(client, '/cabot/event', 'std_msgs/String')
        self.event_topic.subscribe(self._event_callback)

        self.adapter = pygatt.GATTToolBackend()
        self.target = None
        self.last_heartbeat = time.time()

        # speak
        self.alive = False
        self.ready = False

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
                for char in self.chars:
                    if target_chars.get(char.uuid):
                        char.subscribe_to(self.target)
                    else:
                        char.not_found()
                self.ready = True

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
            self.manager.on_terminate(self)

    def buffered_callback(self, handle, value):
        buf = self.data_buffer[handle]
        if value[-1] == ord('\n'):
            result = json.loads(str(buf + value[:-1]))
            # TODO
        else:
            self.data_buffer[handle] = buf + value

    def req_stop(self):
        self.alive = False

    def stop(self):
        self.alive = False
        self.ready = False
        if self.target is not None:
            try:
                self.target.disconnect()
            except pygatt.exceptions.BLEError:
                #device is already closed #logger.info("device disconnected")
                pass
        self.adapter.stop()

    def handleSpeak(self, req):
        if not self.ready:
            return None
        text = req['text']
        force = req['force']
        if force:
            text = "__force_stop__\n" + text
        self.call_async(self.speak_uuid, text)
        return True

    def _event_callback(self, msg):
        event = BaseEvent.parse(msg['data'])
        if event is None:
            logger.error("cabot event %s cannot be parsed", msg['data'])
            return

        if event.type != NavigationEvent.TYPE:
            return

        if event.subtype == "next":
            # notify the phone next event
            self.call_async(self.navi_uuid, "next")

        if event.subtype == "arrived":
            self.call_async(self.navi_uuid, "arrived")

        if event.subtype == "content":
            self.call_async(self.content_uuid, event.param)

        if event.subtype == "sound":
            self.call_async(self.sound_uuid, event.param)


    @util.setInterval(0.01, times=1)
    def call_async(self, uuid, text):
        logger.info("call async %s with %s", uuid, text)
        try:
            self.target.char_write(uuid, value=text.encode("utf-8"))
        except:
            traceback.print_exc()
            try:
                self.target.char_write(uuid, value=text.encode("utf-8"))
            except:
                traceback.print_exc()
                return


class AnyDeviceManager(gatt.DeviceManager, object):
    def __init__(self, adapter_name, name=None):
        super().__init__(adapter_name = adapter_name)
        self.name = "CaBot" + ("-" + name if name is not None else "")
        logger.info("name: %s", self.name)
        self.bles = {}
        self.service = roslibpy.Service(client, '/speak', 'cabot_msgs/Speak')
        self.service.advertise(self.handleSpeak)

    def handleSpeak(self, req, res):
        logger.info("/speak request (%s)", str(req))
        for ble in self.bles.values():
            ble.handleSpeak(req=req)
        res['result'] = True
        return True

    def on_terminate(self, bledev):
        logger.info("terminate %s", bledev.address)
        self.bles.pop(bledev.address)

    def make_device(self, mac_address):
        return gatt.Device(mac_address=mac_address, manager=self)

    def device_discovered(self, device):
        if device.alias() == self.name:
            if not device.mac_address in self.bles.keys():
                ble = CaBotBLE(address=device.mac_address, manager=self)
                self.bles[device.mac_address] = ble
                thread = threading.Thread(target=ble.start)
                thread.start()

    def stop(self):
        for ble in self.bles.values():
            ble.req_stop()


def main():
    cabot_name = os.environ['CABOT_NAME'] if 'CABOT_NAME' in os.environ else None
    adapter_name = os.environ['CABOT_BLE_ADAPTOR'] if 'CABOT_BLE_ADAPTOR' in os.environ else "hci0"

    device_manager = AnyDeviceManager(adapter_name=adapter_name, name=cabot_name)

    # power on the adapter
    if not device_manager.is_adapter_powered:
        device_manager.is_adapter_powered = True

    device_manager.start_discovery(["35CE0000-5E89-4C0D-A3F6-8A6A507C1BF1"])

    try:
        device_manager.run()
    except:
        logger.info(traceback.format_exc())
    finally:
        device_manager.stop()
        device_manager._main_loop.quit()
        client.terminate()

if __name__ == "__main__":
    main()
