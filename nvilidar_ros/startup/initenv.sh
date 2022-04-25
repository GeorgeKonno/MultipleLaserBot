#!/bin/bash
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="2e3c", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout",  SYMLINK+="nvilidar"' >/etc/udev/rules.d/nvilidar_usb.rules

service udev reload
sleep 2
service udev restart

