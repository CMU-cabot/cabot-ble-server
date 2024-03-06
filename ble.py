#!/usr/bin/env python

# Copyright (c) 2023  Carnegie Mellon University
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

import asyncio
from bleak import BleakScanner, BleakClient
import dbus
import gzip
import math
from uuid import UUID
import queue
import time
import traceback
import threading

import common


class BLEDeviceManager:
    def __init__(self, adapter_name, cabot_name, cabot_manager):
        self.adapter_name = adapter_name
        self.cabot_name = cabot_name
        self.cabot_manager = cabot_manager
        self._tasks = set()
        self.bles = {}
        self.alive = True
        self.bles_lock = threading.Lock()

    def add_task(self, coroutine):
        self._tasks.add(asyncio.create_task(coroutine))

    def get_tasks(self):
        if self._tasks:
            common.logger.debug(f"new tasks {self._tasks}")
        running_tasks = [task for task in self._tasks]
        self._tasks.clear()
        return running_tasks

    async def device_discovery(self):
        service_uuids = [str(CABOT_BLE_UUID(0))]
        while self.alive:
            try:
                common.logger.info("discover")
                devices = await BleakScanner.discover(service_uuids=service_uuids)
                if devices:
                    common.logger.info(f"BLE devices are found {devices}")
                    for d in devices:
                        self.device_discovered(d)
                else:
                    common.logger.info("BLE Devices are not found")
                if self.bles:
                    await asyncio.sleep(5)
            except:  # noqa: E722
                await asyncio.sleep(5)

    async def enable_bluetooth(self):
        import subprocess
        while self.alive:
            bus = dbus.SystemBus()
            bluez_adapter = bus.get_object("org.bluez", "/org/bluez/" + self.adapter_name)
            is_adapter_powered = bluez_adapter.Get("org.bluez.Adapter1", "Powered",
                                                   dbus_interface='org.freedesktop.DBus.Properties') == 1

            if is_adapter_powered:
                await asyncio.sleep(5)
                continue

            common.logger.warn("bluetooth is off, sudo rfkill unblock bluetooth")
            result = subprocess.call(["sudo", "rfkill", "unblock", "bluetooth"])
            if result != 0:
                common.logger.error("Could not unblock rfkill bluetooth")
                continue
            await asyncio.sleep(1)

            common.logger.warn("bluetooth is off, sudo systemctl restart bluetooth")
            result = subprocess.call(["sudo", "systemctl", "restart", "bluetooth"])
            if result != 0:
                common.logger.error("Could not restart bluetooth service")
                continue
            await asyncio.sleep(1)

            while not is_adapter_powered and self.alive:
                bus = dbus.SystemBus()
                bluez_adapter = bus.get_object("org.bluez", "/org/bluez/" + self.adapter_name)
                temp = dbus.Interface(bluez_adapter, 'org.freedesktop.DBus.Properties')
                temp.Set("org.bluez.Adapter1", "Powered", True)
                is_adapter_powered = bluez_adapter.Get("org.bluez.Adapter1", "Powered",
                                                       dbus_interface='org.freedesktop.DBus.Properties') == 1
                common.logger.info("Bluetooth is off, so powering on")
                await asyncio.sleep(1)

    async def run(self):
        self.add_task(self.device_discovery())
        self.add_task(self.enable_bluetooth())

        running_tasks = self.get_tasks()
        while self.alive:
            if running_tasks:
                common.logger.debug(f"run tasks {running_tasks}")
                done, running_tasks = await asyncio.wait(running_tasks, timeout=1.0, return_when=asyncio.FIRST_COMPLETED)
                if done:
                    common.logger.debug(f"done task {done}")
            else:
                # TODO
                # need to improve to deal with multiple devices
                # this can still connect with multiple devices if the devices are found at once
                # await asyncio.sleep(1)
                self.alive = False
                continue
            running_tasks.update(self.get_tasks())

    def stop(self):
        self.alive = False

    def device_discovered(self, device):
        if device.name == f"CaBot-{self.cabot_name}":
            if device.address not in self.bles.keys():
                common.logger.debug("device {} {} discovered".format(device.name, device.address))
                ble = CaBotBLE(device=device, ble_manager=self, cabot_manager=self.cabot_manager)
                with self.bles_lock:
                    self.bles[device.address] = ble
                self.add_task(ble.start())
            else:
                pass

    def on_terminate(self, bledev):
        common.logger.info("terminate %s", bledev.address)
        with self.bles_lock:
            self.bles.pop(bledev.address)

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


def CABOT_BLE_UUID(_id):
    return UUID("35CE{0:04X}-5E89-4C0D-A3F6-8A6A507C1BF1".format(_id))


CHAR_WRITE_MAX_SIZE = 512  # should not be exceeded this value


class CaBotBLE:
    def __init__(self, device, ble_manager, cabot_manager):
        self.device = device
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

        self.chars.append(common.CabotLogRequestChar(self, CABOT_BLE_UUID(0x50),
                                                     self.cabot_manager,
                                                     common.CabotLogResponseChar(self, CABOT_BLE_UUID(0x51))))

        self.chars.append(common.HeartbeatChar(self, CABOT_BLE_UUID(0x9999)))

        self.last_heartbeat = time.time()

        # speak
        self.alive = False
        self.ready = False

        self.queue = queue.PriorityQueue()
        self.error_count = 0

    def send_text(self, uuid, text, priority=10):
        data = f"{text}".encode("utf-8")
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

    async def check_queue(self):
        if not self.ready:
            return
        if self.queue.empty():
            return
        common.logger.info("queue size = {}".format(self.queue.qsize()))
        (priority, uuid, data) = self.queue.get()

        start = time.time()
        try:
            total = 0
            for packet in self.make_packets(data, CHAR_WRITE_MAX_SIZE):
                total += len(packet)
                await self.client.write_gatt_char(uuid, packet, True)
            common.logger.info("char_write %d bytes in %f seconds (%.2fKbps)",
                               total, (time.time()-start), total*8/(time.time()-start)/1024)
            self.error_count = 0
        except:  # noqa: E722
            self.error_count += 1
            if self.error_count > 3:
                self.alive = False
            else:
                self.queue.put((priority, uuid, data))
                # common.logger.info(traceback.format_exc())
                common.logger.error("check_queue got an error {}".format(self.error_count))

    async def start(self):
        async with BleakClient(self.device.address) as client:
            class ClientWrapper:
                def __init__(self, ble_manager, client):
                    self.ble_manager = ble_manager
                    self.client = client

                def subscribe(self, uuid, callback, indication):
                    self.ble_manager.add_task(self._subscribe(uuid, callback, indication))

                async def _subscribe(self, uuid, callback, indication):
                    await self.client.start_notify(uuid, callback)
                
            if not client.is_connected:
                common.logger.info("cannot connect")
                return
            self.device_status_char.start()
            self.ros_status_char.start()
            self.battery_status_char.start()
            self.ready = True
            self.alive = True
            self.client = client
            common.logger.info("connected")
            service = client.services.get_service(CABOT_BLE_UUID(0))
            if not service:
                return
            common.logger.info(f"got service {service}")

            wrapper = ClientWrapper(self.ble_manager, client)
            for char in self.chars:
                char.subscribe_to(wrapper)

            common.logger.info("started heart beat")
            try:
                self.version_char.notify()
                common.logger.info("sent version")
            except:  # noqa: E722
                traceback.print_exc()

            self.last_heartbeat = time.time()
            timeout = 3.0
            while client.is_connected and time.time() - self.last_heartbeat < timeout and self.alive:
                await self.check_queue()
                await asyncio.sleep(0.01)
            common.logger.info("client maybe disconnected")
        self.stop()

    def req_stop(self):
        self.alive = False

    def stop(self):
        self.alive = False
        self.ready = False
        self.ble_manager.on_terminate(self)
        self.device_status_char.stop()
        self.ros_status_char.stop()
        self.battery_status_char.stop()
