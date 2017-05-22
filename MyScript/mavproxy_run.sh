#!/bin/bash -e
mavproxy.py --master=/dev/ttyUSB0,57600 --out 0.0.0.0:14550 --out 192.168.2.1:14550