#!/usr/bin/python
import dbus
import dbus.service
import dbus.mainloop.glib

from gi.repository import GLib
import threading
import time
import uuid

from cabot import util

BLUEZ_SERVICE_NAME = 'org.bluez'
BLUEZ_DEVICE_IFACE = 'org.bluez.Device1'
BLUEZ_ADAPTER_IFACE = 'org.bluez.Adapter1'
BLUEZ_GATT_CHAR_IFACE = 'org.bluez.GattCharacteristic1'
DBUS_OM_IFACE =      'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE =    'org.freedesktop.DBus.Properties'

CHECK_DEVICE_INTERVAL = 0.5

class NoPairingAgent(dbus.service.Object):
    def __init__(self, bus):
        self.path = "/org/bluez/no_pairing_agent/{}".format(str(uuid.uuid4()).replace("-","_"))
        dbus.service.Object.__init__(self, bus, self.path)

    @dbus.service.method('org.bluez.Agent1', in_signature="o", out_signature="")
    def RequestAuthorization(self, device):
        print("RequestAuthorization (%s)" % (device))
        return

class Device:
    def __init__(self, bus, path):
        self.bus = bus
        self.path = path
        self.obj = self.bus.get_object("org.bluez", path)
        self.om = self.bus.get_object("org.bluez", "/")

    def alias(self):
        return self.obj.Get(BLUEZ_DEVICE_IFACE, "Alias", dbus_interface=DBUS_PROP_IFACE)

    def pair_reply_cb():
        print("pair replied")

    def pair_error_cb():
        print("pair error")

    @property
    def address(self):
        return self.obj.Get(BLUEZ_DEVICE_IFACE, "Address", dbus_interface=DBUS_PROP_IFACE)

    def connect(self):
        if not self.obj.Get(BLUEZ_DEVICE_IFACE, "Paired", dbus_interface=DBUS_PROP_IFACE):
            print('Pairing with', self.path)
            try:
                self.obj.Pair(reply_handler=self.pair_reply_cb,
                              error_handler=self.pair_error_cb,
                              dbus_interface=BLUEZ_DEVICE_IFACE)
                while True:
                    if self.obj.Get(BLUEZ_DEVICE_IFACE, "Connected", dbus_interface=DBUS_PROP_IFACE):
                        break
                    ### TBD, timeout
            except:
                raise RuntimeError("got error while pairing")
        elif not self.obj.Get(BLUEZ_DEVICE_IFACE, "Connected", dbus_interface=DBUS_PROP_IFACE):
            print('Connecting to', self.path)
            try:
                self.obj.Connect(dbus_interface=BLUEZ_DEVICE_IFACE)
            except:
                raise RuntimeError("Cannot connect to {}".format(self.path))

    def disconnect(self):
        # do nothing
        pass

    def get_characteristic(self, uuid):
        while not self.obj.Get(BLUEZ_DEVICE_IFACE, "ServicesResolved", dbus_interface=DBUS_PROP_IFACE):
            print('waiting services are resolved')
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
                return Characteristic(self.bus, path)

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
            print("unsubscribe", self.path)
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
        print("You need to override {}".format(inspect.currentframe().f_code.co_name))

    """
    It will report all devices including one has already been discovered unlike normal StartDiscovery method on dbus
    It will clear discovery status when you call stop_discovery
    """
    def start_discovery(self, uuids=None):
        print("start_discovery")
        with self.mutex:
            print("start_discovery mutex begin")
            if self.discovery_thread is None:
                self.discovery_uuids = [uuid.lower() for uuid in uuids]
                self.discovery_thread = threading.Thread(target=self.__run_discovery)
                print("starting thread")
                self.discovery_thread.start()
            print("start_discovery mutex end")

    def __run_discovery(self):
        args = {
            "DuplicateData": dbus.types.Boolean(True),
        }            
        if self.discovery_uuids:
            args.update({"UUIDs": self.discovery_uuids})
        self.bluez_adapter.SetDiscoveryFilter(args, dbus_interface=BLUEZ_ADAPTER_IFACE)
        self.bluez_adapter.StartDiscovery(dbus_interface=BLUEZ_ADAPTER_IFACE)
        self.__run_discovery_alive = True
        while self.__run_discovery_alive:
            self.__check_device()
            try: 
                time.sleep(CHECK_DEVICE_INTERVAL)
            except:
                print("stop __run_discovery")
                break

    def __check_device(self):
        print(self.discovered)
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
                
                with self.mutex:
                    print("check_device mutex begin")
                    if path not in self.discovered:
                        self.discovered[path] = self.make_device(path)
                        self._device_discovered(self.discovered[path])
                    print("check_device mutex end")

    def stop_discovery(self):
        print("stop_discovery")
        with self.mutex:
            print("stop_discovery mutex start")
            self.bluez_adapter.StopDiscovery(dbus_interface=BLUEZ_ADAPTER_IFACE)
            self.__run_discovery_alive = False
            self.discovery_thread.join()
            self.discovery_thread = None
            self.discovered = {}
            print("stop_discovery mutex end")

    def run(self):
        self.loop = GLib.MainLoop()
        self.loop.run()


if __name__ == "__main__":
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    loop = GLib.MainLoop()

    class MyDeviceManager(DeviceManager):
        def device_discovered(self, device):
            print(device.path)
    
    dm = MyDeviceManager()

    from uuid import UUID
    CABOT_BLE_UUID = lambda _id: UUID("35CE{0:04X}-5E89-4C0D-A3F6-8A6A507C1BF1".format(_id))
    dm.start_discovery([str(CABOT_BLE_UUID(0))])
    #dm.start_discovery()

    try:
        loop.run()
    except:
        print("stop loop")
        loop.quit()
        dm.stop_discovery()
        import sys
        sys.exit(0)

