#!/usr/bin/env python3

import bluepy.btle

class Delegate(bluepy.btle.DefaultDelegate):
    def handleDiscovery(self, dev, isNewDev, isNweData):
        print("handleDiscovery {}".format(dev.addr))
        for (sdid, desc, value) in dev.getScanData():
            print([desc, value])

device = None
alive = True
delegate = Delegate()

scanner = bluepy.btle.Scanner().withDelegate(delegate)
scanner.clear()
scanner.start(passive=False)

while not device:
    print("process")
    scanner.process(1)
scanner.stop()

def run():

    try:
        print("connecting to {}".format(device.addr))
        conn = bluepy.btle.Peripheral(device.addr, device.addrType)
        print(conn.services)

        for s in conn.services:
            print(s.uuid)
            for c in s.getCharacteristics():
                print(c.uuid)
    finally:
        if conn:
            conn.disconnect()


import threading

t = threading.Thread(target=run)
t.start()

t.join()
