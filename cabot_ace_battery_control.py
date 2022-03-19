#!/usr/bin/python3

import os
import sys
import inspect
import logging
import traceback
import threading
import queue
import struct
import time
import serial

from cabot import util

DEBUG=False

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

class BatteryDriverDelegate(object):
    def battery_info(self, info):
        logger.error("{} is not implemented".format(inspect.currentframe().f_code.co_name))


class BatteryDriver:
    def __init__(self, port_name, baud, delegate=None, timeout=1):
        self.port_name = port_name
        self.baud = baud
        self.delegate = delegate
        self.timeout = timeout

        self.port = None
        self.write_thread = None
        self.write_queue = queue.Queue()
        self.write_lock = threading.RLock()
        self.read_lock = threading.RLock()
        self.is_alive = True


    def start(self):
        sleep_time = 3
        while True:
            try:
                driver = None
                logger.info("opening serial %s", self.port_name)
                while True:
                    try:
                        self.port = serial.Serial(self.port_name, self.baud, timeout=5, write_timeout=10)
                        break
                    except serial.SerialException as e:
                        logging.error("%s", e)
                        time.sleep(3)
                logger.info("serial port opened");
                self._run()
            except KeyboardInterrupt as e:
                logger.info("KeyboardInterrupt")
                break
            except serial.SerialException as e:
                error_msg = str(e)
                logger.error(e)
                time.sleep(sleep_time)
            except OSError as e:
                error_msg = str(e)
                logger.error(e)
                traceback.print_exc(file=sys.stdout)
                time.sleep(sleep_time)
            except IOError as e:
                error_msg = str(e)
                logger.error(e)
                time.sleep(sleep_time)
            except termios.error as e:
                error_msg = str(e)
                logger.error("connection disconnected")
                time.sleep(sleep_time)
            except SystemExit as e:
                break
            except:
                logger.error(sys.exc_info()[0])
                traceback.print_exc(file=sys.stdout)
                sys.exit()
            finally:
                if driver:
                    driver.stop()


    def stop(self):
        self.is_alive = False


    def checksum(data):
        temp = 0
        for d in data:
            temp += d
        return 0xFF - (0xFF & temp)


    def data_send_start_command(self):
        self.send_command(0x31) # '1'


    def data_send_stop_command(self):
        self.send_command(0x32) # '2'


    def turn_jetson_switch_on(self):
        self.send_command(0x33) # '3'


    def set_12v_power(self, on_off):
        self.send_command(0x34, on_off) # '4' ON(1) OFF(0)


    def set_5v_power(self, on_off):
        self.send_command(0x35, on_off) # '5' ON(1) OFF(0)


    def set_odrive_power(self, on_off):
        self.send_command(0x36, on_off) # '6' ON(1) OFF(0)


    def shutdown(self):
        self.send_command(0x37) # '7'


    def set_lowpower_shutdown_threshold(self, threshold):
        self.send_command(0x38, threshold) # '8' 0~100(%)


    def send_command(self, command, arg0=0x00):
        data = bytearray(13)
        data[0] = 0xAA
        data[1] = 0xAA
        data[2] = command
        data[3] = 0x08
        data[4] = arg0
        data[12] = BatteryDriver.checksum(data[0:12])
        self.write_queue.put(bytes(data))


    def _process_write_queue(self):
        while self.is_alive:
            if self.write_queue.empty():
                time.sleep(0.01)
            else:
                data = self.write_queue.get()
                while True:
                    try:
                        if isinstance(data, bytes):
                            self._write(data)
                        else:
                            logger.error("Trying to write invalid data type: %s" % type(data))
                        break
                    except serial.SerialTimeoutException as exc:
                        logger.error('Write timeout: %s' % exc)
                        time.sleep(1)
                    except RuntimeError as exc:
                        logger.error('Write thread exception: %s' % exc)
                        break


    def _write(self, data):
        with self.write_lock:
            self.port.write(data)


    def _tryRead(self, length):
        try:
            bytes_remaining = length
            result = bytearray()
            read_start = time.time()
            while bytes_remaining != 0 and time.time() - read_start < self.timeout:
                with self.read_lock:
                    received = self.port.read(bytes_remaining)
                if len(received) != 0:
                    result.extend(received)
                    bytes_remaining -= len(received)
            if bytes_remaining != 0:
                raise IOError("Returned short (expected %d bytes, received %d instead)." % (length, length - bytes_remaining))
            return bytes(result)
        except Exception as e:
            raise IOError("Serial Port read failure: %s" % e)


    def _run(self):
        if self.write_thread is None:
            self.write_thread = threading.Thread(target=self._process_write_queue)
            self.write_thread.daemon = True
            self.write_thread.start()            

        self.data_send_stop_command()
        time.sleep(1)
        self.data_send_start_command()

        header_count = 0
        state = 0
        while self.is_alive:
            time.sleep(0.001)
            if self.port.inWaiting() < 1:
                continue
            received = self._tryRead(1)
            if state == 0:  # searching header
                if received[0] == 0xAA:
                    header_count += 1
                else:
                    header_count = 0
                if header_count == 2:
                    header_count = 0
                    state = 1
            if state == 1: # read command
                cmd = self._tryRead(2)
                if cmd[0] == 0x30: # fixed command
                    data = self._tryRead(cmd[1])
                    checksum = int.from_bytes(self._tryRead(1), 'little')
                    if checksum == BatteryDriver.checksum(data):
                        info = BatteryInfo(data)
                        #logger.info(info)
                        if self.delegate:
                            self.delegate.battery_info(info)
                state = 0



class BatteryInfo:
    def __init__(self, data):
        (power_jetson, power_12v, power_5v, power_odrive, battery_capacity) = struct.unpack('BBBBB', data[0:5])
        (jetson_current, loop_cnt) = struct.unpack('<HI', data[5:11])
        (shutdown, lowpower_shutdown) = struct.unpack('BB', data[11:13])
        # do something here
        self.power_jetson = power_jetson
        self.power_12v = power_12v
        self.power_5v = power_5v
        self.power_odrive = power_odrive
        self.battery_capacity = battery_capacity
        self.jetson_current = jetson_current
        self.loop_cnt = loop_cnt
        self.shutdown = shutdown
        self.lowpower_shutdown = lowpower_shutdown

    def __str__(self):
        temp = ""
        for key in self.__dict__:
            temp += "{}: {}\n".format(key, self.__dict__[key])
        return temp.strip()


def main():
    port_name = os.environ['CABOT_ACE_BATTERY_PORT'] if 'CABOT_ACE_BATTERY_PORT' in os.environ else '/dev/ttyACM0'
    baud = int(os.environ['CABOT_ACE_BATTERY_BAUD']) if 'CABOT_ACE_BATTERY_BAUD' in os.environ else 115200
    BatteryDriver(port_name, baud).start()

if __name__ == "__main__":
    logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    main()
