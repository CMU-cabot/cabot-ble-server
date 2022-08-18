#!/usr/bin/python
import dbus
import dbus.service
import dbus.mainloop.glib

from gi.repository import GLib
import threading
import time
import traceback
import uuid
import logging

from cabot import util

DEBUG=True
logging.basicConfig(format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

BLUEZ_SERVICE_NAME = 'org.bluez'
BLUEZ_DEVICE_IFACE = 'org.bluez.Device1'
BLUEZ_ADAPTER_IFACE = 'org.bluez.Adapter1'
BLUEZ_GATT_CHAR_IFACE = 'org.bluez.GattCharacteristic1'
DBUS_OM_IFACE =      'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE =    'org.freedesktop.DBus.Properties'

CHECK_DEVICE_INTERVAL = 1.0

class NoPairingAgent(dbus.service.Object):
    def __init__(self, bus):
        self.path = "/org/bluez/no_pairing_agent/{}".format(str(uuid.uuid4()).replace("-","_"))
        dbus.service.Object.__init__(self, bus, self.path)

    @dbus.service.method('org.bluez.Agent1', in_signature="o", out_signature="")
    def RequestAuthorization(self, device):
        logger.info("RequestAuthorization (%s)" % (device))
        return

class Device:
    def __init__(self, bus, path):
        self.bus = bus
        self.path = path
        self.obj = self.bus.get_object("org.bluez", path)
        self.om = self.bus.get_object("org.bluez", "/")
        self.chars = {}

    @property
    def alias(self):
        try:
            return self.obj.Get(BLUEZ_DEVICE_IFACE, "Alias", dbus_interface=DBUS_PROP_IFACE)
        except:
            return None

    @property
    def name(self):
        try:
            return self.obj.Get(BLUEZ_DEVICE_IFACE, "Name", dbus_interface=DBUS_PROP_IFACE)
        except:
            return None

    def pair_reply_cb(self):
        logger.info("pair replied")

    def pair_error_cb(self, error):
        logger.info("pair error")

    @property
    def address(self):
        try:
            return self.obj.Get(BLUEZ_DEVICE_IFACE, "Address", dbus_interface=DBUS_PROP_IFACE)
        except:
            return None

    @property
    def connected(self):
        try:
            return self.obj.Get(BLUEZ_DEVICE_IFACE, "Connected", dbus_interface=DBUS_PROP_IFACE)
        except:
            return None

    def connect(self):
        if not self.obj.Get(BLUEZ_DEVICE_IFACE, "Paired", dbus_interface=DBUS_PROP_IFACE):
            logger.info('Pairing with %s', self.path)
            try:
                self.obj.Pair(reply_handler=self.pair_reply_cb,
                              error_handler=self.pair_error_cb,
                              dbus_interface=BLUEZ_DEVICE_IFACE)
                while True:
                    if self.obj.Get(BLUEZ_DEVICE_IFACE, "Connected", dbus_interface=DBUS_PROP_IFACE):
                        break
                    ### TBD, timeout
                return True
            except Exception as e:
                logger.error(e)
                return False
        elif not self.obj.Get(BLUEZ_DEVICE_IFACE, "Connected", dbus_interface=DBUS_PROP_IFACE):
            logger.info('Connecting to %s', self.path)
            try:
                self.obj.Connect(dbus_interface=BLUEZ_DEVICE_IFACE)
                return True
            except Exception as e:
                logger.error(e)
                return False

    def disconnect(self):
        logger.info("Disconnecting")
        try:
            self.obj.Disconnect(dbus_interface=BLUEZ_DEVICE_IFACE)
        except:
            logger.info(traceback.format_exc())


    def get_characteristic(self, uuid):
        while not self.obj.Get(BLUEZ_DEVICE_IFACE, "ServicesResolved", dbus_interface=DBUS_PROP_IFACE):
            logger.info('waiting services are resolved')
            time.sleep(1)
        
        objects = self.om.GetManagedObjects(dbus_interface=DBUS_OM_IFACE)
        for path, interfaces in objects.items():
            if not path.startswith(self.path):
                continue
            if BLUEZ_GATT_CHAR_IFACE not in interfaces.keys():
                continue
            obj = self.bus.get_object("org.bluez", path)
            char_uuid = obj.Get(BLUEZ_GATT_CHAR_IFACE, "UUID", dbus_interface=DBUS_PROP_IFACE)
            if str(char_uuid) == str(uuid):
                if str(uuid) not in self.chars:
                    self.chars[str(uuid)] = Characteristic(self.bus, path)
                return self.chars[str(uuid)]

    def get_handle(self, uuid):
        return self.get_characteristic(uuid)

    def char_write_handle(self, handle, value, wait_for_response, timeout):
        handle.write(value)

class Characteristic:
    def __init__(self, bus, path):
        self.bus = bus
        self.path = path
        self.obj = self.bus.get_object("org.bluez", path)
        self.signal = None
        self.callbacks = []

    def properties_changed_cb(self, iface, changed_props, invalidated_props):
        if iface == BLUEZ_GATT_CHAR_IFACE and "Value" in changed_props:
            value = bytearray()
            for val in changed_props["Value"]:
                value.extend(val.to_bytes(1, 'little'))
            
            for callback in self.callbacks:
                callback(self, value)

    def subscribe(self, uuid, callback, indication):
        self.callbacks.append(callback)
        self.prop_iface = dbus.Interface(self.obj, DBUS_PROP_IFACE)
        self.signal = self.prop_iface.connect_to_signal("PropertiesChanged",
                                                        self.properties_changed_cb)
        
        self.obj.StartNotify(dbus_interface=BLUEZ_GATT_CHAR_IFACE)

    def stop(self):
        if self.signal:
            logger.info("unsubscribe %s", self.path)
            self.signal.remove()
        self.callbacks = []

    def write(self, value):
        dvalue = dbus.Array([dbus.Byte(b) for b in value], "ay")
        options = dbus.types.Dictionary({}, "sv")
        self.obj.WriteValue(dvalue, options, dbus_interface=BLUEZ_GATT_CHAR_IFACE)


class DeviceManager:
    def __init__(self, adapter_name="hci0"):
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        self.adapter_name = adapter_name
        self.mutex = threading.Lock()
        self.discovery_thread = None
        
        bus = dbus.SystemBus()
        self.bus = bus
        self.bluez_root = bus.get_object("org.bluez", "/")
        self.bluez_adapter = bus.get_object("org.bluez", "/org/bluez/" + adapter_name)
        self.discovered = {}

        agent = NoPairingAgent(self.bus)
        bluez_object = self.bus.get_object("org.bluez", "/org/bluez")
        agent_manager = dbus.Interface(bluez_object, 'org.bluez.AgentManager1')
        agent_manager.RegisterAgent(agent.path, "NoInputNoOutput")
        agent_manager.RequestDefaultAgent(agent.path)

    @property
    def is_adapter_powered(self) -> bool:
        return self.bluez_adapter.Get("org.bluez.Adapter1", "Powered",
                                      dbus_interface='org.freedesktop.DBus.Properties') == 1

    @is_adapter_powered.setter
    def is_adapter_powered(self, value):
        temp = dbus.Interface(self.bluez_adapter, 'org.freedesktop.DBus.Properties')
        temp.Set("org.bluez.Adapter1", "Powered", value)

    def make_device(self, path):
        return Device(self.bus, path)

    @util.setInterval(0.01, times=1)
    def _device_discovered(self, device):
        self.device_discovered(device)

    def device_discovered(self, device):
        import inspect
        logger.info("You need to override {}".format(inspect.currentframe().f_code.co_name))

    """
    It will report all devices including one has already been discovered unlike normal StartDiscovery method on dbus
    It will clear discovery status when you call stop_discovery
    """
    @util.setInterval(0.1, times=1)
    def start_discovery(self, uuids=None):
        logger.info("start_discovery")
        with self.mutex:
            logger.info("start_discovery mutex begin")
            if self.discovery_thread is None:
                self.discovery_uuids = [uuid.lower() for uuid in uuids]
                self.discovery_thread = threading.Thread(target=self.__run_discovery)
                logger.info("starting thread")
                self.discovery_thread.start()
            logger.info("start_discovery mutex end")

    def __run_discovery(self):
        args = {
            "DuplicateData": dbus.types.Boolean(True),
        }            
        if self.discovery_uuids:
            args.update({"UUIDs": self.discovery_uuids})
        self.bluez_adapter.SetDiscoveryFilter(args, dbus_interface=BLUEZ_ADAPTER_IFACE)
        self.bluez_adapter.StartDiscovery(dbus_interface=BLUEZ_ADAPTER_IFACE)
        self.__run_discovery_alive = True
        count = 3
        while self.__run_discovery_alive:
            self.__check_device()
            try: 
                time.sleep(CHECK_DEVICE_INTERVAL)
                count -= 1
                if count < 0:
                    count = 3
                    self.discovered.clear()
            except:
                logger.info(traceback.format_exc())
                logger.info("stop __run_discovery")
                break

    def __check_device(self):
        #logger.info(self.discovered)
        objects = self.bluez_root.GetManagedObjects(dbus_interface=DBUS_OM_IFACE)
        for path, interfaces in objects.items():
            if BLUEZ_DEVICE_IFACE not in interfaces.keys():
                continue
            
            if path.startswith("/org/bluez/{}/".format(self.adapter_name)):
                obj = self.bus.get_object("org.bluez", path)
                UUIDs = obj.Get("org.bluez.Device1", "UUIDs", dbus_interface=DBUS_PROP_IFACE)
                result = True
                if self.discovery_uuids:
                    for uuid in self.discovery_uuids:
                        result = result and (uuid in UUIDs)
                if not result:
                    continue
                
                if path not in self.discovered:
                    with self.mutex:
                        self.discovered[path] = self.make_device(path)
                        self._device_discovered(self.discovered[path])
                        #self._device_discovered(self.make_device(path))

    def stop_discovery(self):
        logger.info("stop_discovery")
        with self.mutex:
            logger.info("stop_discovery mutex start")
            self.bluez_adapter.StopDiscovery(dbus_interface=BLUEZ_ADAPTER_IFACE)
            self.__run_discovery_alive = False
            self.discovery_thread.join()
            self.discovery_thread = None
            self.discovered = {}
            logger.info("stop_discovery mutex end")

    def run(self):
        self.loop = GLib.MainLoop()
        self.loop.run()

    def stop(self):
        self.loop.quit()


if __name__ == "__main__":
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    loop = GLib.MainLoop()

    class MyDeviceManager(DeviceManager):
        def device_discovered(self, device):
            logger.info(device.path)
    
    dm = MyDeviceManager()

    from uuid import UUID
    CABOT_BLE_UUID = lambda _id: UUID("35CE{0:04X}-5E89-4C0D-A3F6-8A6A507C1BF1".format(_id))
    dm.start_discovery([str(CABOT_BLE_UUID(0))])
    #dm.start_discovery()

    try:
        loop.run()
    except:
        logger.info("stop loop")
        loop.quit()
        dm.stop_discovery()
        import sys
        sys.exit(0)

