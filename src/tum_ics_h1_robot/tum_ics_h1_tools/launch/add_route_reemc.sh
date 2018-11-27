#!/bin/bash
sudo route add -net 10.68.0.0 netmask 255.255.255.0 gw 192.168.4.1 dev $1

