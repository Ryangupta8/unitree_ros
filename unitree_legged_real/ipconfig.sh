#!/bin/bash

sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

sudo ifconfig enx3c8cf8ff943a down
sudo ifconfig enx3c8cf8ff943a up 192.168.123.161 netmask 255.255.255.0 # 192.168.123.161

