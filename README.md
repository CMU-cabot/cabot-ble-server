## For development on macos (will not be merged)

- Tested with Docker Desktop
- Support only TCP transport
- cabot-ios-app [CaBotAppModel.socketPort](https://github.com/CMU-cabot/cabot-ios-app/blob/86bd67beceae80a87f478a33993d7db622a3c443/CaBot/CaBotAppModel.swift#L401) should be 5001
- Use `./launch.sh` to launch the server
```
git submodule update --init --recursive
docker-compose build --build-arg UID=$UID
./launch.sh
```
- `./install.sh` does not work on macos

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
CABOT_NO_BLE                # default is false, do not use BLE if true
NO_BUILD                    # default is false, do not build when launch if true
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
# User name to login jetson (default=cabot)
CABOT_JETSON_USER=cabot
# specify jetson mate config. expects a config which is provided for cabot
# jetson hosts will be automaticallly shutdown in the written order
CABOT_JETSON_CONFIG="D:192.168.1.52:rs3 D:192.168.1.51:rs2 D:192.168.1.50:rs1"
# ssh id file to log in jetson
CABOT_ID_FILE=id_ed25519_cabot
```

On Jetson hosts, please use "visudo" command to modify "/etc/sudoer" and add the following line to shutdown Jetson automatically.
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

