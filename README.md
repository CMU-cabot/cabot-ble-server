# CaBot BLE Server

BLE server to monitor/control [CaBot](https://github.com/cmu-cabot/cabot)

# Install

- edit `.env` file

```
git submodule update --init --recursive
docker-compose build --build-arg UID=$UID
./install.sh
```

# Uninstall

```
./uninstall.sh
```


# Environment Variables (see .env file as example)
```
CABOT_NAME                  # cabot name
CABOT_BLE_ADAPTER           # default is 'hci0'
CABOT_START_AT_LAUNCH       # default is 0, launch cabot system if 1 at start up
```

## variables for cabot2-ace battery configuration
```
CABOT_ACE_BATTERY_PORT=/dev/ttyESP32
CABOT_ACE_BATTERY_BAUD=115200
```

## variables for device check
```
LIDAR_IF=enp8s0
LIDAR_IP=192.168.2.201
MICRO_CONTROLLER=Arduino
```

### related to model variant
- GPU PC + Realsense x 3
```
CABOT_REALSENSE_SERIAL_1
CABOT_REALSENSE_SERIAL_2
CABOT_REALSENSE_SERIAL_3
CABOT_CAMERA_NAME_1
CABOT_CAMERA_NAME_2
CABOT_CAMERA_NAME_3
```
- NUC + Jetson (Realsense x 3)
```
CABOT_JETSON_CONFIG="D:192.168.1.51:rs1 D:192.168.1.52:rs2 D:192.168.1.53:rs3"
```

## variable to power off remote hosts by the power off button
```
CABOT_REMOTE_POWEROFF_CONFIG
# specify remote hosts in the order to power off by the power off button
# write in the format <user name>@<host name>,
# and separate config for multiple hosts by space
# ex) "cabot@192.158.1.52 cabot@192.158.1.51 cabot@192.158.1.50"
```

Please modify your "~/.ssh/config", and make make sure you can connect from docker contaier to remote hosts by ssh without password.
An example setting is below.
```
Host 192.168.1.*
    IdentityFile ~/.ssh/id_ed25519_cabot
```

In remote hosts, please use "visudo" command to modify "/etc/sudoer" and add the following line.
```
cabot   ALL=NOPASSWD: /sbin/poweroff
```

##


# Test
- launch ble server
```
docker-compose build --build-arg UID=$UID
docker-compose up ble
```

- launch cabot_ace battery driver alone
```
CABOT_ACE_BATTERY_PORT=/dev/ttyXXXX CABOT_ACE_BATTERY_BAUD=115200 python3 -m cabot_ace.cabot_ace_battery_driver

options
-od,　　--odrive_power    #　turn on[1] or off[0] odrive motor controller.
```

