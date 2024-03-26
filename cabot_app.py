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

import asyncio
import os
import time
import json
import threading
import traceback
import signal
import subprocess
import sys
from rosidl_runtime_py.convert import message_to_ordereddict

import rclpy

import common
import ble
import tcp

from cabot import util
from cabot_ace import BatteryDriverNode, BatteryDriver, BatteryDriverDelegate
from cabot_log_report import LogReport
from cabot_msgs.srv import Speak

MTU_SIZE = 2**10  # could be 2**15, but 2**10 is enough
CHAR_WRITE_MAX_SIZE = 512  # should not be exceeded this value
WAIT_AFTER_CONNECTION = 0.25  # wait a bit after connection to avoid error


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
            common.logger.info(traceback.format_exc())

    def set_clients(self, clients):
        if len(clients) == 0:
            device = {
                'type': "User App",
                'model': "cabot-ios-app",
                'level': "Error",
                'message': "disconnected",
                'values': []
            }
            self.devices.append(device)
        else:
            client = clients[0]
            device = {
                'type': "User App",
                'model': "cabot-ios-app",
                'level': "OK" if client.connected else "Error",
                'message': "connected" if client.connected else "disconnected",
                'values': []
            }
            self.devices.append(device)

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

class AppClient():
    ALIVE_THRESHOLD = 3.0

    def __init__(self, id):
        self.id = id
        self.address = None
        self.type = None
        self.last_updated = time.time()

    @property
    def connected(self):
        return time.time() - self.last_updated < AppClient.ALIVE_THRESHOLD

    def __str__(self):
        return f"AppClient: address={self.address}, type={self.type}"

class CaBotManager(BatteryDriverDelegate):
    def __init__(self, jetson_poweroff_commands=None):
        self._device_status = DeviceStatus()
        self._cabot_system_status = SystemStatus()
        self._battery_status = None
        self._log_report = LogReport()
        self.systemctl_lock = threading.Lock()
        self.start_flag = False
        self.stop_run = None
        self.check_interval = 5
        self.run_count = 0
        self._jetson_poweroff_commands = jetson_poweroff_commands
        self._client_map = {}

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
            common.logger.info("shutdown requested")
            self.stop()
            self.poweroffPC()
    # BatteryDriverDelegate end

    def add_log_request(self, request, callback):
        self._log_report.add_to_queue(request, callback)

    @util.setInterval(1, logger=common.logger)
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
                common.logger.info("Start at launch is requested, but device is not OK")

    def _check_device_status(self):
        if self._cabot_system_status.is_active():
            result = self._runprocess(["sudo", "-E", "./cabot-device-check/check_device_status.sh", "-j", "-s"])
        else:
            result = self._runprocess(["sudo", "-E", "./cabot-device-check/check_device_status.sh", "-j"])
        if result and result.returncode == 0:
            self._device_status.ok()
        else:
            self._device_status.error()
        self._device_status.set_json(result.stdout)
        self._device_status.set_clients(self.get_clients_by_type("Normal"))

    def _check_service_active(self):
        result = self._runprocess(["systemctl", "--user", "is-active", "cabot"])
        common.logger.info(f"_check_service_active {result}")
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
                common.logger.info("check_service_active unknown status: %s", result.stdout.strip())

        #global diagnostics
        self._cabot_system_status.set_diagnostics(common.diagnostics)
        common.diagnostics = []

    def _runprocess(self, command):
        return subprocess.run(command, capture_output=True, text=True, env=os.environ.copy())

    def _call(self, command, lock=None):
        result = 0
        if lock is not None and not lock.acquire(blocking=False):
            common.logger.info("lock could not be acquired")
            return result
        try:
            common.logger.info("calling %s", str(command))
            result = subprocess.call(command)
        except:
            common.logger.error(traceback.format_exc())
        finally:
            if lock is not None:
                lock.release()
        return result

    def rebootPC(self):
        self._call(["sudo", "systemctl", "reboot"], lock=self.systemctl_lock)

    def poweroffPC(self):
        if self._jetson_poweroff_commands is not None:
            for command in self._jetson_poweroff_commands:
                common.logger.info("send shutdown request to jetson: %s", str(command))
                self._call(command, lock=self.systemctl_lock)

        self._call(["sudo", "systemctl", "poweroff"], lock=self.systemctl_lock)

    def startCaBot(self):
        self._call(["systemctl", "--user", "start", "cabot"], lock=self.systemctl_lock)
        self._cabot_system_status.activating()

    def stopCaBot(self):
        self._call(["systemctl", "--user", "stop", "cabot"], lock=self.systemctl_lock)
        self._cabot_system_status.deactivating()

    def device_status(self):
        return self._device_status

    def cabot_system_status(self):
        return self._cabot_system_status

    def cabot_battery_status(self):
        return self._battery_status

    def register_client(self, id, address=None, client_type=None):
        if id is None:
            return
        client = AppClient(id)
        if id in self._client_map:
            client = self._client_map[id]
        else:
            self._client_map[id] = client
        client.last_updated = time.time()
        if address is not None and client.address is None:
            client.address = address
            common.logger.info(client)
        if client_type is not None and client.type is None:
            client.type = client_type
            common.logger.info(client)
        return client

    def get_client(self, id):
        if id in self._client_map:
            return self._client_map[id]
        return None

    def get_clients_by_type(self, client_type):
        clients = []
        for key in self._client_map.keys():
            if self._client_map[key].type != client_type:
                continue
            clients.append(self._client_map[key])
        return clients


quit_flag=False
tcp_server = None
ble_manager = None
driver = None
battery_thread = None
tcp_server_thread = None

def get_thread_traceback(thread_id):
    frame = sys._current_frames().get(thread_id)
    if frame:
        return ''.join(traceback.format_stack(frame))
    else:
        return "Thread not found"

import ctypes
import threading

def terminate_thread(thread):
    if not thread.is_alive():
        return
    exc = ctypes.py_object(SystemExit)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread.ident), exc)
    if res == 0:
        raise ValueError("Invalid thread ID")
    elif res > 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(thread.ident, 0)
        raise SystemError("PyThreadState_SetAsyncExc failed")

def sigint_handler(sig, frame):
    common.logger.info("sigint_handler")
    global quit_flag
    if sig == signal.SIGINT:
        try:
            quit_flag = True
            if driver:
                driver.stop()
                battery_thread.join()
            if ble_manager:
                ble_manager.stop()

            if tcp_server_thread:
                try:
                    terminate_thread(tcp_server_thread)
                except:
                    common.logger.error(traceback.format_exc())
                while tcp_server_thread.is_alive():
                    common.logger.info(f"wait tcp server thread {tcp_server_thread}")
                    tcp_server_thread.join(timeout=1)

            if common.ros2_thread:
                try:
                    rclpy.shutdown()
                except:
                    common.logger.error(traceback.format_exc())
                while common.ros2_thread.is_alive():
                    common.logger.info(f"wait ros2 server thread {common.ros2_thread}")
                    common.ros2_thread.join(timeout=1)
        except:
            common.logger.error(traceback.format_exc())
    else:
        common.logger.error("Unexpected signal")


async def main():
    signal.signal(signal.SIGINT, sigint_handler)
    cabot_name = os.environ['CABOT_NAME'] if 'CABOT_NAME' in os.environ else None
    adapter_name = os.environ['CABOT_BLE_ADAPTER'] if 'CABOT_BLE_ADAPTER' in os.environ else "hci0"
    start_at_launch = (os.environ['CABOT_START_AT_LAUNCH'] == "1") if 'CABOT_START_AT_LAUNCH' in os.environ else False

    port_name = os.environ['CABOT_ACE_BATTERY_PORT'] if 'CABOT_ACE_BATTERY_PORT' in os.environ else None
    baud = int(os.environ['CABOT_ACE_BATTERY_BAUD']) if 'CABOT_ACE_BATTERY_BAUD' in os.environ else None

    no_ble = os.environ['CABOT_NO_BLE']=='true' if 'CABOT_NO_BLE' in os.environ else False
    no_tcp = os.environ['CABOT_NO_TCP']=='true' if 'CABOT_NO_TCP' in os.environ else False
    common.logger.info(f"No BLE = {no_ble}, No TCP = {no_tcp}")

    cabot_manager = CaBotManager()
    jetson_poweroff_commands = None
    jetson_user = os.environ['CABOT_JETSON_USER'] if 'CABOT_JETSON_USER' in os.environ else "cabot"
    jetson_config = os.environ['CABOT_JETSON_CONFIG'] if 'CABOT_JETSON_CONFIG' in os.environ else None
    if jetson_config is not None:
        id_file = os.environ['CABOT_ID_FILE'] if 'CABOT_ID_FILE' in os.environ else ""
        id_dir = os.environ['CABOT_ID_DIR'] if 'CABOT_ID_DIR' in os.environ else ""
        id_file_path = os.path.join(id_dir, id_file)
        if not os.path.exists(id_file_path):
            common.logger.error("ssh id file does not exist '{}'".format(id_file_path))
            sys.exit()

        jetson_poweroff_commands = []
        items = jetson_config.split()
        for item in items:
            split_item = item.split(':')
            if len(split_item)!=3:
                common.logger.error("Invalid value of CABOT_JETSON_CONFIG is found '{}'".format(item))
                sys.exit()

            result = subprocess.call(["ssh", "-i", id_file_path, jetson_user + "@" + split_item[1], "exit"])
            if result != 0:
                common.logger.error("Cannot connect Jetson host, please check ssh config. user:{}, host:{}, ssh key file:{}".format(jetson_user, split_item[1], id_file_path))
                continue

            result = subprocess.call(["ssh", "-i", id_file_path, jetson_user + "@" + split_item[1], "sudo poweroff -w"])
            if result != 0:
                common.logger.error("Cannot call poweroff on Jetson host, please check sudoer config. user:{}, host:{}".format(jetson_user, split_item[1], id_file_path))
                continue

            jetson_poweroff_commands.append(["ssh", "-i", id_file_path, jetson_user + "@" + split_item[1], "sudo poweroff"])

    cabot_manager = CaBotManager(jetson_poweroff_commands=jetson_poweroff_commands)
    cabot_manager.run(start=start_at_launch)

    global driver
    global battery_thread
    if port_name is not None and baud is not None:
        driver = BatteryDriver(port_name, baud, delegate=cabot_manager)
        battery_driver_node = BatteryDriverNode(driver)
        battery_thread = threading.Thread(target=driver.start)
        battery_thread.start()
        battery_thread_node = threading.Thread(target=battery_driver_node.start)
        battery_thread_node.start()

    try:
        result = subprocess.call(["grep", "-E", "^ControllerMode *= *le$", "/etc/bluetooth/main.conf"])
        if result != 0:
            common.logger.error("Please check your /etc/bluetooth/main.conf")
            line = subprocess.check_output(["grep", "-E", "ControllerMode", "/etc/bluetooth/main.conf"])
            common.logger.error("Your ControllerMode is '{}'".format(line.decode('utf-8').replace('\n', '')))
            common.logger.error("Please use ./setup_bluetooth_conf.sh to configure LE mode")
            sys.exit(result)
    except:
        pass

    global tcp_server
    global ble_manager
    global quit_flag

    def handleSpeak(req, res):
        res.result = False
        req_dictionary = message_to_ordereddict(req)
        req_dictionary['request_id'] = time.clock_gettime_ns(time.CLOCK_REALTIME)
        if ble_manager:
            ble_manager.handleSpeak(req_dictionary, res)
        if tcp_server:
            tcp_server.handleSpeak(req_dictionary, res)
        return res

    common.cabot_node_common.create_service(Speak, '/speak', handleSpeak)

    global tcp_server_thread
    try:
        if not no_tcp:
            if tcp_server is None:
                tcp_server = tcp.CaBotTCP(cabot_manager=cabot_manager)
                common.add_event_handler(tcp_server)
                if True:
                    tcp_server_thread = threading.Thread(target=tcp_server.start)
                    tcp_server_thread.start()
                else:
                    tcp_server.start()

        if not no_ble:
            if ble_manager is not None:
                common.remove_event_handler(ble_manager)
            ble_manager = ble.BLEDeviceManager(adapter_name=adapter_name, cabot_name=cabot_name, cabot_manager=cabot_manager)
            common.add_event_handler(ble_manager)
            await ble_manager.run()
        else:
            while not quit_flag:
                time.sleep(1)
    except KeyboardInterrupt:
        common.logger.info("keyboard interrupt")
    except:
        common.logger.info(traceback.format_exc())
    cabot_manager.stop()
    common.logger.info("exiting the app")
    sys.exit(0)

if __name__ == "__main__":
    asyncio.run(main())
