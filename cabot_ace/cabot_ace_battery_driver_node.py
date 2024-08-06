#!/usr/bin/python3

import os
import logging
import time
import rclpy
import rclpy.node
from std_srvs.srv import Empty, SetBool
import traceback

from .cabot_ace_battery_driver import BatteryDriver, BatteryDriverDelegate, BatteryStatus
from cabot import util

DEBUG=False

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

class Delegate(BatteryDriverDelegate):
    def __init__(self):
        self.count=0

    def battery_status(self, status):
        self.count+=1
        if self.count % 50 == 0:
            logger.info(info)
        if status.shutdown:
            logger.info("shutdown requested")
        if status.lowpower_shutdown:
            logger.info("lowpower shutdown requested")


class BatteryDriverNode(rclpy.node.Node):
    def __init__(self, driver):
        super().__init__("battery_driver_node", start_parameter_services=False)
        self.driver = driver
        self.connected = False

        self.service0 = self.create_service(Empty, '/ace_battery_control/turn_jetson_switch_on', self.turn_jetson_switch_on)
        self.service1 = self.create_service(SetBool, '/ace_battery_control/set_12v_power', self.set_12v_power)
        self.service2 = self.create_service(SetBool, '/ace_battery_control/set_5v_power', self.set_5v_power)
        self.service3 = self.create_service(SetBool, '/ace_battery_control/set_odrive_power', self.set_odrive_power)
        self.service4 = self.create_service(Empty, '/ace_battery_control/shutdown', self.shutdown)
        #self.service5 = self.create_service(cabot_msgs.msg.SetUInt8, '/ace_battery_control/set_lowpower_shutdown_threshold', self.set_lowpower_shutdown_threshold)

    def start(self):
        try:
            rclpy.spin(self)
        except:
            logger.error(traceback.format_exc())

    def turn_jetson_switch_on(self, req, res):
        self.driver.turn_jetson_switch_on()
        return res

    def set_12v_power(self, req, res):
        if req.data:
            self.driver.set_12v_power(1)
        else:
            self.driver.set_12v_power(0)
        res.success = True
        return res

    def set_5v_power(self, req, res):
        if req.data:
            self.driver.set_5v_power(1)
        else:
            self.driver.set_5v_power(0)
        res.success = True
        return res

    def set_odrive_power(self, req, res):
        if req.data:
            self.driver.set_odrive_power(1)
        else:
            self.driver.set_odrive_power(0)
        res.success = True
        return res

    def shutdown(self, req, res):
        self.driver.shutdown()
        return res

    def set_lowpower_shutdown_threshold(self, req, res):
        self.driver.set_lowpower_shutdown_threshold(msg['value'])
        return res

def main():
    rclpy.init()
    
    port_name = os.environ['CABOT_ACE_BATTERY_PORT'] if 'CABOT_ACE_BATTERY_PORT' in os.environ else '/dev/ttyACM0'
    baud = int(os.environ['CABOT_ACE_BATTERY_BAUD']) if 'CABOT_ACE_BATTERY_BAUD' in os.environ else 115200
    delegate = Delegate()
    driver = BatteryDriver(port_name, baud, delegate=delegate)
    node = BatteryDriverNode(driver)

    try:
        rclpy.spin(node)
    except:
        logger.error(traceback.format_exc())
    rclpy.destroy_node(node)

if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    main()
