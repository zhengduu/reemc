#!/bin/bash

sudo service ntp stop
sleep 2
sudo ntpdate -u basestation
sleep 2
sudo service ntp start
sleep 2
ntpq -p
