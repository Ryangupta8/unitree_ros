#!/bin/bash

sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

sudo ifconfig enp60s0 down
sudo ifconfig enp60s0 up 192.168.123.162 netmask 255.255.255.0
#sudo ifconfig enp60s0 down # enpxxx is your PC usb port
# sudo ifconfig enp60s0 192.168.123.162
# sudo ifconfig enp60s0 up
