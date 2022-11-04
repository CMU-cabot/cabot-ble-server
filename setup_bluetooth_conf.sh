#!/bin/bash


grep -E "^ControllerMode *= *le$" /etc/bluetooth/main.conf
if [ $? -eq 0 ]; then
    echo "Your /etc/bluetooth/main.conf looks good"
else
    sudo cp main.conf /etc/bluetooth/main.conf
fi

echo "sudo systemctl daemon-reload"
sudo systemctl daemon-reload
echo "sudo systemctl restart bluetooth"
sudo systemctl restart bluetooth

