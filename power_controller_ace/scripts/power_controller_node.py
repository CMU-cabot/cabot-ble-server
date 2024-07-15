#!/usr/bin/python3

import os
import json
import logging
import rclpy
import rclpy.node
from std_srvs.srv import Empty, SetBool
import std_msgs.msg
import threading
import traceback

from power_controller_ace.driver import BatteryDriver, BatteryDriverDelegate, BatteryStatus

DEBUG=False

logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

class BatteryDriverNode(rclpy.node.Node):
    def __init__(self, driver):
        super().__init__("battery_driver_node", start_parameter_services=False)
        self.driver = driver
        self.connected = False

        self.service0 = self.create_service(Empty, 'turn_jetson_switch_on', self.turn_jetson_switch_on)
        self.service1 = self.create_service(SetBool, 'set_12v_power', self.set_12v_power)
        self.service2 = self.create_service(SetBool, 'set_5v_power', self.set_5v_power)
        self.service3 = self.create_service(SetBool, 'set_odrive_power', self.set_odrive_power)
        self.service4 = self.create_service(Empty, 'shutdown', self.shutdown)
        #self.service5 = self.create_service(cabot_msgs.msg.SetUInt8, '/ace_battery_control/set_lowpower_shutdown_threshold', self.set_lowpower_shutdown_threshold)
        self.status_pub = self.create_publisher(std_msgs.msg.String, "/ace_battery_status", 10) # TODO: temporal - should be fixed

    def battery_status(self, status: BatteryStatus):
        msg = std_msgs.msg.String()
        msg.data = json.dumps(status.json)
        self.status_pub.publish(msg)
        if status.shutdown:
            logger.info("shutdown requested")
        if status.lowpower_shutdown:
            logger.info("lowpower shutdown requested")

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
    driver = BatteryDriver(port_name, baud)
    battery_thread = threading.Thread(target=driver.start)
    battery_thread.start()
    node = BatteryDriverNode(driver)
    driver.delegate = node

    logger.info(f"{port_name=}, {baud=}")

    try:
        rclpy.spin(node)
    except:
        logger.error(traceback.format_exc())
    driver.stop()
    battery_thread.join()

if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    main()
