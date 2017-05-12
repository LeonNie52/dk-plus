#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2017/3/5
# @Author  : Leon.Nie
# @Site    : 
# @File    : Demo_formation.py

"""
This script is to test the APF method for only one UAV.
"""

import sys, time, os

sys.path.append("..")

if not os.path.exists('log'):
    os.mkdir('log')

from dronekit import connect
from drone_network import Networking
from collision_avoidance import CollisionThread
from act_tool import arm_and_takeoff
import numpy as np

import logging
import logging.config

logging.config.fileConfig("../logging.conf")
logger = logging.getLogger()

# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# connection_string = 'tcp:192.168.6.46:5763'
# connection_string = 'tcp:192.168.6.111:5763'

# Connect to the Vehicle
logger.info('Connecting to vehicle on: %s', connection_string)
vehicle = connect(connection_string, wait_ready=True)

while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print " Waiting for home location ..."
# We have a home location, so print it!
logger.info("Home location: %s", vehicle.home_location)

# Create the interface with UDP broadcast sockets

address = ("192.168.6.255", 54545)
network = Networking(address, "UDP_BROADCAST", vehicle)

# Add collision avoidance algorithm
single = False
t_collision = CollisionThread(network, algorithm='formation', single=single)

# Get all vehicle attributes (state)
print "\nGet all vehicle attribute values:"
logger.info(" Autopilot Firmware version: %s", vehicle.version)
logger.info("System ID：%s", vehicle.parameters['SYSID_THISMAV'])

# Set the targetLocation for the team, Heading South

# Team Home Location
# Playground
lat = 39.979352
lon = 116.339748

# Football Field
# lat = 39.9790234
# lon = 116.3407892

# Formation
formation_set = np.array([[-20, 0, 20],
                          [0, 0, 0],
                          [0, 0, 0]], dtype=float)
# formation_set = np.array([[-25, 25],
#                           [0, 0],
#                           [0, 0]], dtype=float)

t_collision.formation.setFormation(lat, lon, formation_set)

t_collision.formation.set_target_Loc(alt=15, dNorth=-100, dEast=0)

logger.info("Initializing interface")
network.run()

arm_and_takeoff(vehicle, 15)

logger.info("Starting collision avoidance scheme")
t_collision.start()

while not t_collision.formation.target_reached:
    pass

t_collision.changeMode("POSHOLD")

logger.info("Hold Position for %s seconds", 10)
time.sleep(10)

t_collision.formation.ChangetoHome()

t_collision.changeMode("GUIDED")

# while not t_collision.formation.target_reached and not t_collision.formation.home_returned:
#     pass

# t_collision.formation.set_owntarget_Loc(vehicle.home_location)
#
# while not t_collision.formation.ownhome_returned:
#     pass

print "Press any key to exit script"
exit = raw_input()
