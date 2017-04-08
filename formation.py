#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time    : 2017/3/5
# @Author  : Leon.Nie
# @Site    : 
# @File    : formation.py
"""
Drones proceed in formation base on the algorithm Artificial Potential Field
"""
from geo_tools import *
import numpy as np
import logging


class Formation:
    def __init__(self, network):
        self.network = network
        self.MaxLeadForce = 10
        self.MaxForce = 10

        self.safeDistance_K = 0.6
        self.leadForce_K = 1.5
        self.dampForce_K = -1.0
        # self.FormationForce_K = 0.5 * 10e3
        self.FormationForce_K = 0.1
        self.AvoidanceForce_K = 2.0

        self.TeamHomeLocation = None
        self.targetLocation = None
        self.FormationPosition = None

        self.target_reached = False

        self.home_returned = False

        self.distancePrecise = 5  # in meters

    def setFormation(self, formation_set):
        self.FormationPosition = np.matrix(formation_set)

    def set_target_Loc(self, lat, lon, alt, dNorth, dEast):
        """
        Target Location. Set before taking off
        :param lat: original latitude
        :param lon: original longitude
        :param alt: Target altitude
        :param dNorth:
        :param dEast:
        :return:
        """
        self.TeamHomeLocation = get_location_metres(lat,
                                                    lon,
                                                    0, 0, 0)

        self.targetLocation = get_location_metres(lat,
                                                  lon,
                                                  alt, dNorth, dEast)

        logging.info("Target Location set: %s", self.targetLocation)

    def get_target_Loc(self):
        return self.targetLocation

    def get_distance2target(self):
        return get_distance_metres(self.network.vehicle_params.global_lat,
                                   self.network.vehicle_params.global_lon,
                                   self.targetLocation.lat,
                                   self.targetLocation.lon)

    def getPosition(self, teammate):
        """
        Return self target position in Team
        :param teammate: 
        :return: 
        """
        x, y, z = self.get_cenPos(teammate)

        Vx, Vy, Vz = self.get_cenVel(teammate)

        if Vx > 0:
            theta = math.pi / 2 - math.atan(Vy / Vx)
        elif Vx < 0:
            theta = - math.pi / 2 - math.atan(Vy / Vx)
        else:
            if Vy > 0:
                theta = 0
            else:
                theta = math.pi

        # phi = math.atan(Vz / math.sqrt(Vx ** 2 + Vy ** 2))
        phi = 0

        Rotaz = np.matrix(
            [[math.cos(theta), math.sin(theta), 0],
             [-math.sin(theta), math.cos(theta), 0],
             [0, 0, 1]])

        Rotax = np.matrix(
            [[1, 0, 0],
             [0, math.cos(phi), -math.sin(phi)],
             [0, math.sin(phi), math.cos(phi)]])

        c = self.FormationPosition[:, int(self.network.vehicle_params.SYSID_THISMAV - 1)].reshape(3, 1)
        abPos = np.array(Rotaz * Rotax * c).ravel()

        # logging.debug("Local Formation Position : %s", abPos)

        Pos = np.array(get_location_formation(x,
                                              y,
                                              z, abPos[0], abPos[1], abPos[2]))
        # logging.debug("Global Formation Position : %s", Pos)

        return Pos

    def get_cenPos(self, teammate):
        """
         Nearly proximate because we are running in a quite small range
        :param teammate:
        :return:
        """
        lat = self.network.vehicle_params.global_lat
        lon = self.network.vehicle_params.global_lon
        alt = self.network.vehicle_params.global_alt
        for drone in teammate:
            lat += drone.global_lat
            lon += drone.global_lon
            alt += drone.global_alt
        c_lat = lat / float(len(teammate) + 1)
        c_lon = lon / float(len(teammate) + 1)
        c_alt = alt / float(len(teammate) + 1)
        return c_lat, c_lon, c_alt

    def get_cenVel(self, teammate):
        velocity = self.network.vehicle_params.velocity
        for drone in teammate:
            velocity = [drone.velocity[i] + velocity[i] for i in range(len(velocity))]
        cenVel = [x / float(len(teammate) + 1) for x in velocity]

        return cenVel

    def DampForce(self):
        dampForce = self.dampForce_K * np.array(self.network.vehicle_params.velocity)

        return dampForce

    def LeadForce(self, teammate, single):
        if len(teammate) == 0 or single:
            cenPos_NED = get_location_NED(self.TeamHomeLocation,
                                          self.network.vehicle_params.global_lat,
                                          self.network.vehicle_params.global_lon,
                                          self.network.vehicle_params.global_alt)

            # cenPos = np.array([self.network.vehicle_params.global_lat,
            #                    self.network.vehicle_params.global_lon])
            # cenAlt = np.array([self.network.vehicle_params.global_alt])
        else:
            cenPos_NED = get_location_NED(self.TeamHomeLocation,
                                          self.get_cenPos(teammate)[0],
                                          self.get_cenPos(teammate)[1],
                                          self.get_cenPos(teammate)[2])

            # cenPos = self.get_cenPos(teammate)[0:2]
            # cenAlt = self.get_cenPos(teammate)[-1]

        cenPos = np.array([cenPos_NED.north, cenPos_NED.east])
        tarPos_NED = get_location_NED(self.TeamHomeLocation,
                                      self.targetLocation.lat,
                                      self.targetLocation.lon,
                                      self.targetLocation.alt)
        tarPos = np.array([tarPos_NED.north, tarPos_NED.east])
        # tarPos = np.array([self.targetLocation.lat,
        #                    self.targetLocation.lon])
        #
        # tarAlt = np.array([self.targetLocation.lat])

        logging.debug("Center_Position: %s ; Target_Position: %s ;", cenPos, tarPos)

        leadforce = self.leadForce_K * (tarPos - cenPos) / np.linalg.norm(tarPos - cenPos)

        if np.linalg.norm(leadforce) > self.MaxLeadForce:
            leadforce = leadforce * self.MaxLeadForce / np.linalg.norm(leadforce)

        # For now ,no force on altitude
        leadforce = np.append(leadforce, np.zeros(1, ))

        return leadforce

    def FormationForce(self, teammate, single):
        if len(teammate) == 0 or single:
            FormationForce = 0
        else:
            ownPos_NED = get_location_NED(self.TeamHomeLocation,
                                          self.network.vehicle_params.global_lat,
                                          self.network.vehicle_params.global_lon,
                                          self.network.vehicle_params.global_alt)
            ownPos = np.array([ownPos_NED.north, ownPos_NED.east])

            ownAlt = np.array([ownPos_NED.down])

            forPos = self.getPosition(teammate)

            formationPos_NED = get_location_NED(self.TeamHomeLocation,
                                                forPos[0], forPos[1], forPos[2])

            Formation_position = np.array([formationPos_NED.north,
                                           formationPos_NED.east,
                                           formationPos_NED.down])

            # FormationForce = self.FormationForce_K * (Formation_position[0:2] - ownPos) / np.linalg.norm(
            #     Formation_position[0:2] - ownPos)
            FormationForce = self.FormationForce_K * (Formation_position[0:2] - ownPos)
            # For now ,no force on altitude
            FormationForce = np.append(FormationForce, [0])

            logging.debug("Own Position: %s", np.hstack((ownPos, ownAlt)))

        return FormationForce

    def AvoidanceForce(self, teammate, single):
        """
        Improved version of repulsive force in APF
        :param teammate:
        :param single:
        :return:
        """
        force = 0
        if len(teammate) == 0 or single:
            return force
        else:

            ownPos_NED = get_location_NED(self.TeamHomeLocation,
                                          self.network.vehicle_params.global_lat,
                                          self.network.vehicle_params.global_lon,
                                          self.network.vehicle_params.global_alt)
            ownPos = np.array([ownPos_NED.north, ownPos_NED.east])
            ownVel = np.array(self.network.vehicle_params.velocity)

            for drone in teammate:

                objPos_NED = get_location_NED(self.TeamHomeLocation,
                                              drone.global_lat,
                                              drone.global_lon,
                                              drone.global_alt)
                objPos = np.array([objPos_NED.north, objPos_NED.east])
                #
                # distanceWGS = get_distance_metres(self.network.vehicle_params.global_lat,
                #                                   self.network.vehicle_params.global_lon,
                #                                   drone.global_lat,
                #                                   drone.global_lon)

                distanceNED = get_distance_NED(objPos_NED, ownPos_NED)

                objVel = np.array(drone.velocity)

                relvel = (ownVel - objVel)[0:2]  # no force on the altitude
                dis = objPos - ownPos

                val = np.dot(relvel, dis) / np.linalg.norm(dis)

                if val > 0:
                    Rsav = self.network.FORMATION_NEAR_ZONE + self.safeDistance_K  * val
                else:
                    Rsav = self.network.FORMATION_NEAR_ZONE  # original version

                if distanceNED > Rsav:
                    single_force = 0  # avoid force generated by single object
                else:

                    angle = self.calcos(relvel, dis)

                    single_force_K = self.AvoidanceForce_K * Rsav * (1 / distanceNED - 1 / Rsav)

                    single_force_ = -single_force_K * dis / np.linalg.norm(dis)

                    single_force = single_force_ + (relvel / np.linalg.norm(relvel)
                                                    * np.linalg.norm(single_force_) * np.cos(angle))
                    single_force *= val

                    # For now ,no force on altitude
                    single_force = np.append(single_force, np.zeros(1, ))

                    logging.info("Drone: %s too close. Avoidance Force Activated!!!",
                                 drone.SYSID_THISMAV)
                    logging.debug("single Avoidance Force: %s", single_force)
                    logging.debug("Safety distance: %s", Rsav)
                force += single_force

            return force

    def origin_AvoidanceForce(self, teammate, single):
        """
        Temporary the original version of repulsive force in APF
        :param teammate:
        :param single:
        :return:
        """
        force = 0
        if len(teammate) == 0 or single:
            return force
        else:
            ownPos = np.array([self.network.vehicle_params.global_lat,
                               self.network.vehicle_params.global_lon])

            Rsav = self.network.FORMATION_NEAR_ZONE  # original version

            for drone in teammate:
                distance = get_distance_metres(self.network.vehicle_params.global_lat,
                                               self.network.vehicle_params.global_lon,
                                               drone.global_lat,
                                               drone.global_lon)
                if distance > Rsav:
                    single_force = 0  # avoid force generated by single object
                else:
                    objPos = np.array([drone.global_lat, drone.global_lon])

                    single_force_K = -self.AvoidanceForce_K * Rsav * (1.0 / distance - 1.0 / Rsav)

                    single_force = single_force_K * (objPos - ownPos) / np.linalg.norm(objPos - ownPos)

                    # For now ,no force on altitude
                    single_force = np.append(single_force, np.zeros(1, ))

                    logging.info("Drone: %s too close. Avoidance Force Activated!!!",
                                 drone.SYSID_THISMAV)
                    logging.debug("original single Avoidance Force: %s", single_force)

                force += single_force

            return force

    def calcos(self, x, y):
        """
        calculate the angle between two vectors
        :param x: ndarray
        :param y: ndarray
        :return:
        """
        cos_angle = np.dot(x, y) / (np.linalg.norm(x) * np.linalg.norm(y))
        angle = np.arccos(cos_angle)
        return angle

    def TotalForce(self, teammate, single):

        LeadForce = self.LeadForce(teammate, single)
        FormationForce = self.FormationForce(teammate, single)
        AvoidanceForce = self.AvoidanceForce(teammate, single)
        # AvoidanceForce = self.origin_AvoidanceForce(teammate, single)
        DampForce = self.DampForce()

        force = LeadForce + FormationForce + AvoidanceForce + DampForce

        logging.debug("Lead force: %s", LeadForce)
        logging.debug("Formation force: %s ", FormationForce)
        logging.debug("Avoidance Force: %s", AvoidanceForce)
        logging.debug("Damp Force: %s", DampForce)

        if np.linalg.norm(force) > self.MaxForce:
            force = force * self.MaxForce / np.linalg.norm(force)

        total_force = AvoidanceForce + force
        logging.debug("Total force: %s ", total_force)
        return total_force

    def SendVelocity(self, teammate, single):
        add_vel = self.TotalForce(teammate, single) * self.network.POLL_RATE * 10
        # add_vel = self.TotalForce(teammate, single) * 0.1
        velocity = np.array(self.network.vehicle_params.velocity) + add_vel

        # For now Vz=0
        velocity[-1] = 0.0

        logging.debug("Current Velociy: %s", self.network.vehicle_params.velocity)
        logging.debug("Add velocity: %s ", add_vel)

        return list(velocity)

    def reachTarget(self, teammate, single):
        if not self.target_reached:
            if single:
                if get_distance_metres(self.network.vehicle_params.global_lat,
                                       self.network.vehicle_params.global_lon,
                                       self.targetLocation.lat,
                                       self.targetLocation.lon) < self.distancePrecise:

                    self.target_reached = True
                    logging.info("Reach the Target Location!!")

                    return True
                else:
                    return False
            else:
                if get_distance_metres(self.get_cenPos(teammate)[0],
                                       self.get_cenPos(teammate)[1],
                                       self.targetLocation.lat,
                                       self.targetLocation.lon) < self.distancePrecise:

                    self.target_reached = True
                    logging.info("Reach the Target Location!!")

                    return True
                else:
                    return False
        else:
            return self.reachHome(teammate, single)

    def reachHome(self, teammate, single):
        if single:
            if get_distance_metres(self.network.vehicle_params.global_lat,
                                   self.network.vehicle_params.global_lon,
                                   self.TeamHomeLocation.lat,
                                   self.TeamHomeLocation.lon) < self.distancePrecise:
                self.home_returned = True
                logging.info("Reach  Home Location!!")
                return True
            else:
                return False
        else:
            if get_distance_metres(self.get_cenPos(teammate)[0],
                                   self.get_cenPos(teammate)[1],
                                   self.TeamHomeLocation.lat,
                                   self.TeamHomeLocation.lon) < self.distancePrecise:
                self.home_returned = True
                logging.info("Reach  Home Location!!")
                return True
            else:
                return False

    def ChangetoHome(self):
        """
        Change target to home location
        :return:
        """
        self.targetLocation = self.TeamHomeLocation
        logging.info("Return to home Location")
