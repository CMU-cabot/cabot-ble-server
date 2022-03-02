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

client = roslibpy.Ros(host='localhost', port=9091)
connected = False
@util.setInterval(1.0)
def polling_ros():
    global connected
    if not client.is_connected:
        if connected:
            logger.info("ROS bridge is not connected")
            connected = False

        logger.debug("polling")
        try:
            client.run(1.0)
            logger.info("ROS bridge is connected")
            connected = True
        except:
            pass

polling_ros()

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
#logger.setLevel(logging.DEBUG)


### Debug
if False:
    from logging import StreamHandler, Formatter

    stream_handler = StreamHandler()
    handler_format = Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    stream_handler.setFormatter(handler_format)
    stream_handler.setLevel(logging.DEBUG)

    for key in logging.Logger.manager.loggerDict:
        logger = logging.Logger.manager.loggerDict[key]
        try:
            logger.setLevel(logging.DEBUG)
            logger.addHandler(stream_handler)
        except:
            pass


class CaBotBLE:
    UUID_FORMAT = "35CE{0:04X}-5E89-4C0D-A3F6-8A6A507C1BF1"

    def __init__(self, address, mgr):
        self.address = address
        self.mgr = mgr
        self.summon_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x09))
        self.dest_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x10))
        self.speak_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x200))
        self.navi_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x300))
        self.content_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x400))
        self.sound_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x500))
        self.heartbeat_uuid = UUID(CaBotBLE.UUID_FORMAT.format(0x9999))
        self.data_buffer = {}

        self.eventTopic = roslibpy.Topic(client, '/cabot/event', 'std_msgs/String')
        self.eventTopic.subscribe(self._event_callback)

        self.adapter = pygatt.GATTToolBackend()
        self.target = None
        self.last_heartbeat = time.time()

        # speak
        self.alive = False
        self.ready = False

    def start(self):
        self.alive = True
        self.start_time = time.time()
        try:
            while time.time() - self.start_time < 60*10 and self.alive:
                self.adapter.start(reset_on_start=False)
                target = None

                try:
                    logger.info("trying to connect to %s" % (self.address))
                    target = self.adapter.connect(self.address, timeout=15, address_type=pygatt.BLEAddressType.random)
                    target.exchange_mtu(64)
                except pygatt.exceptions.NotConnectedError:
                    logger.error("device not connected %s" % (self.address))
                except pygatt.exceptions.NotificationTimeout:
                    logger.error("setting exchange_mtu failed %s" % (self.address))
                    target = None

                if target is not None:
                    try:
                        self.target = target

                        try:
                            self.target.subscribe(self.dest_uuid, self.destination_callback, indication=False)
                            logger.info("subscribed to destination %s" % (self.address))
                        except:
                            logger.info("could not connect to destination %s" % (self.address))
                            return

                        try:
                            self.target.subscribe(self.summon_uuid, self.summons_callback, indication=False)
                            logger.info("subscribed to summons %s" % (self.address))
                        except:
                            logger.info("could not connect to destination %s" % (self.address))
                            return

                        try:
                            self.target.subscribe(self.heartbeat_uuid, self.heartbeat_callback, indication=False)
                            logger.info("subscribed to heartbeat %s" % (self.address))
                        except:
                            logger.info("could not connect to hertbeat %s" % (self.address))
                            return

                        self.ready = True

                        self.last_heartbeat = time.time()
                        timeout = 5.0
                        while time.time() - self.last_heartbeat < timeout and self.alive:
                            if time.time() - self.last_heartbeat > timeout/2.0:
                                logger.info(
                                    "Reconnecting in %.1f seconds %s" % (timeout - (time.time() - self.last_heartbeat), self.address))
                            time.sleep(0.5)
                    except pygatt.exceptions.BLEError:
                        logger.info("device disconnected")
        except:
            logger.error(traceback.format_exc())
        finally:
            self.stop()
            self.mgr.on_terminate(self)

    def buffered_callback(self, handle, value):
        buf = self.data_buffer[handle]

        if value[-1] == ord('\n'):
            result = json.loads(str(buf + value[:-1]))
            # TODO
        else:
            self.data_buffer[handle] = buf + value

    def destination_callback(self, handle, value):
        value = value.decode("utf-8")
        logger.info("destination_callback {}".format(value))

        if value == "__cancel__":
            logger.info("cancel navigation")
            event = NavigationEvent(subtype="cancel", param=None)
            self.eventTopic.publish(roslibpy.Message({'data': str(event)}))
            return

        logger.info("destination: " + value)
        event = NavigationEvent(subtype="destination", param=value)
        self.eventTopic.publish(roslibpy.Message({'data': str(event)}))

    def summons_callback(self, handle, value):
        value = value.decode("utf-8")
        logger.info("summons_callback {}".format(value))
        event = NavigationEvent(subtype="summons", param=value)
        self.eventTopic.publish(roslibpy.Message({'data': str(event)}))

    def heartbeat_callback(self, handle, value):
        #logger.info(("heartbeat(%s):" % self.address) +  str(value))
        self.last_heartbeat = time.time()

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


class AnyDevice(gatt.Device,object):
    """
    An implementation of ``gatt.Device`` that connects to any GATT device
    and prints all services and characteristics.
    """

    def __init__(self, mac_address, manager, auto_reconnect=False):
        super(AnyDevice,self).__init__(mac_address=mac_address, manager=manager)
        self.auto_reconnect = auto_reconnect

    def connect(self):
        print("Connecting...")
        super(AnyDevice,self).connect()

    def connect_succeeded(self):
        super(AnyDevice,self).connect_succeeded()
        print("[%s] Connected" % (self.mac_address))

    def connect_failed(self, error):
        super(AnyDevice,self).connect_failed(error)
        print("[%s] Connection failed: %s" % (self.mac_address, str(error)))

    def disconnect_succeeded(self):
        super(AnyDevice,self).disconnect_succeeded()

        print("[%s] Disconnected" % (self.mac_address))
        if self.auto_reconnect:
            self.connect()

    def services_resolved(self):
        super(AnyDevice,self).services_resolved()

        print("[%s] Resolved services" % (self.mac_address))
        for service in self.services:
            print("[%s]  Service [%s]" % (self.mac_address, service.uuid))
            for characteristic in service.characteristics:
                print("[%s]    Characteristic [%s]" % (self.mac_address, characteristic.uuid))

class AnyDeviceManager(gatt.DeviceManager, object):
    def __init__(self, adapter_name, name=None):
        super(AnyDeviceManager, self).__init__(adapter_name = adapter_name)
        self.name = "CaBot" + ("-" + name if name is not None else "")
        print("name: " + self.name)
        self.bles = {}
        self.service = roslibpy.Service(client, '/speak', 'cabot_msgs/Speak')
        self.service.advertise(self.handleSpeak)

        self.service = roslibpy.Service(client, '/restart', 'std_srvs/Trigger')
        self.service.advertise(self.handleRestart)
        self.service = roslibpy.Service(client, '/reboot', 'std_srvs/Trigger')
        self.service.advertise(self.handleReboot)
        self.service = roslibpy.Service(client, '/poweroff', 'std_srvs/Trigger')
        self.service.advertise(self.handlePoweroff)

    def handleRestart(self, req, res):
        subprocess.call(["systemctl", "--user", "restart", "cabot"])
        res['success']=True
        return True

    def handleReboot(self, req, res):
        subprocess.call(["sudo", "systemctl", "reboot"])
        res['success']=True
        return True

    def handlePoweroff(self, req, res):
        subprocess.call(["sudo", "systemctl", "poweroff"])
        res['success']=True
        return True

    def handleSpeak(self, req, res):
        logger.info("/speak request (%s)"%(str(req)))
        for ble in self.bles.values():
            ble.handleSpeak(req=req)
        res['result'] = True
        return True

    def on_terminate(self, bledev):
        print("terminate {}".format(bledev.address))
        self.bles.pop(bledev.address)

    def make_device(self, mac_address):
        return AnyDevice(mac_address=mac_address, manager=self)

    def device_discovered(self, device):
        if device.alias() == self.name:
            if not device.mac_address in self.bles.keys():
                for service in device.services:
                    print("[%s]  Service [%s]" % (device.mac_address, service.uuid))
                ble = CaBotBLE(address=device.mac_address, mgr=self)
                self.bles[device.mac_address] = ble
                thread = threading.Thread(target=ble.start)
                thread.start()

    def stop(self):
        for ble in self.bles.values():
            ble.req_stop()


if __name__ == "__main__":
    name = os.environ['CABOT_NAME'] if 'CABOT_NAME' in os.environ else None
    print(name)
    adapter_name = "hci0"

    manager = AnyDeviceManager(adapter_name=adapter_name, name=name)

    # power on the adapter
    if not manager.is_adapter_powered:
        manager.is_adapter_powered = True

    manager.start_discovery(service_uuids=["35CE0000-5E89-4C0D-A3F6-8A6A507C1BF1"])

    try:
        manager.run()
    except:
        print(traceback.format_exc())
        logger.info(traceback.format_exc())
    finally:
        manager.stop()
        manager._main_loop.quit()
        client.terminate()
