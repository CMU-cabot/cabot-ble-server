#!/usr/bin/python3

import os
import logging
import time

import roslibpy

from cabot_ace_battery_control import BatteryDriver, BatteryDriverDelegate, BatteryInfo
from cabot import util

DEBUG=False

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

class Delegate(BatteryDriverDelegate):
    def __init__(self):
        self.count=0

    def battery_info(self, info):
        self.count+=1
        if self.count % 50 == 0:
            logger.info(info)
        if info.shutdown:
            logger.info("shutdown requested")
        if info.lowpower_shutdown:
            logger.info("lowpower shutdown requested")


class BatteryControlNode:
    def __init__(self, client, control):
        self.client = client
        self.control = control
        self.connected = False

        self.service0 = roslibpy.Service(self.client, '/ace_battery_control/turn_jetson_switch_on', 'std_srvs/Empty')
        self.service1 = roslibpy.Service(self.client, '/ace_battery_control/set_12v_power', 'std_srvs/SetBool')
        self.service2 = roslibpy.Service(self.client, '/ace_battery_control/set_5v_power', 'std_srvs/SetBool')
        self.service3 = roslibpy.Service(self.client, '/ace_battery_control/set_odrive_power', 'std_srvs/SetBool')
        self.service4 = roslibpy.Service(self.client, '/ace_battery_control/shutdown', 'std_srvs/Empty')
        #self.service5 = roslibpy.Service(self.client, '/ace_battery_control/set_lowpower_shutdown_threshold', 'cabot_msgs/SetUInt8')
        self.polling_ros()

    @util.setInterval(1.0)
    def polling_ros(self):
        if not self.client.is_connected:
            time.sleep(1)
            self.connected = False
        else:
            if not self.connected:
                logger.info("advertise services")
                self.init_services()
                self.connected = True

    def init_services(self):
        self.service0.advertise(self.turn_jetson_switch_on)
        self.service1.advertise(self.set_12v_power)
        self.service2.advertise(self.set_5v_power)
        self.service3.advertise(self.set_odrive_power)
        self.service4.advertise(self.shutdown)
        #self.service5.advertise(self.set_lowpower_shutdown_threshold)

    def turn_jetson_switch_on(self, req, res):
        self.control.turn_jetson_switch_on()
        return True

    def set_12v_power(self, req, res):
        if req['data']:
            self.control.set_12v_power(1)
        else:
            self.control.set_12v_power(0)
        res['success'] = True
        return True

    def set_5v_power(self, req, res):
        if req['data']:
            self.control.set_5v_power(1)
        else:
            self.control.set_5v_power(0)
        res['success'] = True
        return True

    def set_odrive_power(self, req, res):
        if req['data']:
            self.control.set_odrive_power(1)
        else:
            self.control.set_odrive_power(0)
        res['success'] = True
        return True

    def shutdown(self, req, res):
        self.control.shutdown()
        return True

    def set_lowpower_shutdown_threshold(self, req, res):
        self.control.set_lowpower_shutdown_threshold(msg['value'])
        res['success'] = True
        return True

def main():
    client = roslibpy.Ros(host='localhost', port=9091)
    
    port_name = os.environ['CABOT_ACE_BATTERY_PORT'] if 'CABOT_ACE_BATTERY_PORT' in os.environ else '/dev/ttyACM0'
    baud = int(os.environ['CABOT_ACE_BATTERY_BAUD']) if 'CABOT_ACE_BATTERY_BAUD' in os.environ else 115200
    delegate = Delegate()
    control = BatteryDriver(port_name, baud, delegate=delegate)
    BatteryControlNode(client, control)
    
    client.run()
    control.start()


if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    main()
