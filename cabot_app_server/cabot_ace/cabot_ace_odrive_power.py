#!/usr/bin/python3

import os
import sys
import roslibpy

args = sys.argv

client = roslibpy.Ros(host='localhost', port=9091)
client.run()

service = roslibpy.Service(client, '/ace_battery_control/set_odrive_power', 'std_srvs/SetBool')
request = roslibpy.ServiceRequest({'data': False if 1 < len(args) and '0' == args[1] else True})

print ('setting odrive power: {}'.format(request))
result = service.call(request)
print('response: {}'.format(result))

client.terminate()


