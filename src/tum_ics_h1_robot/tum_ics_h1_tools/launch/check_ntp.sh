#!/bin/bash

sudo service ntp stop
sudo service ntp start
ntpq -p
