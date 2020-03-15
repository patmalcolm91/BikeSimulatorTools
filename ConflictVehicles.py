"""
Module for inserting a single vehicle timed to create a conflict with the ego vehicle at an intersection.
The vehicle will be sent at a time which will bring it to the end of the first lane in its route (+/- an offset)
when the ego vehicle is projected to reach a specified target point.

Author: Patrick Malcolm
"""

import traci
import traci.constants as tc
import numpy as np
import xml.etree.ElementTree as ET
from . import RouteTools


class ConflictVehicle:
    def __init__(self, name, typeID, routeID, ego_target, conflict_target_offset=0, release_point=20):
        """
        Initializes a ConflictVehicle object.
        :param name: ID of the vehicle to be inserted
        :param typeID: vehicle type ID of the vehicle to be inserted
        :param routeID: route ID of the route of the vehicle to be inserted
        :param ego_target: coordinate of ego vehicle target point
        :param conflict_target_offset: offset (m) to apply to the conflict vehicle's end-of-first-lane target point
        :param release_point: distance (m) from ego target after which conflict vehicle is no longer altered
        :type name: str
        :type typeID: str
        :type routeID: str
        :type ego_target: (float, float)
        :type conflict_target_offset: float
        :type release_point: float
        """
        self.name = name
        self.typeID = typeID
        self.routeID = routeID
        self.ego_target = ego_target
        self.conflict_target_offset = conflict_target_offset
        self.conflict_edge = traci.route.getEdges(self.routeID)[0]
        self.conflict_lane = RouteTools.get_rightmost_allowed_lane(self.conflict_edge, self.typeID)
        self.conflict_lane_length = traci.lane.getLength(self.conflict_lane) + self.conflict_target_offset
        self.conflict_lane_speed = traci.lane.getMaxSpeed(self.conflict_lane)
        self.deployed = False
        self.done = False
        self.release_point = release_point

    def check(self, ego_pos, ego_speed):
        """
        Project the ETA of the ego vehicle to the target point and if it's within range, deploy the conflict vehicle.
        :param ego_pos: (x, y) coordinate of ego vehicle
        :param ego_speed: speed of ego vehicle (m/s)
        :return: None
        """
        # Calculate ego and conflict vehicles' ETAs
        if ego_speed == 0:
            ego_speed = 0.0001  # avoid division by zero
        ego_dist = np.linalg.norm(np.array(self.ego_target) - np.array(ego_pos))
        ego_eta = ego_dist / ego_speed
        conflict_eta_total = self.conflict_lane_length / self.conflict_lane_speed
        # Check if the conflict vehicle should be deployed
        if not self.deployed and ego_eta <= conflict_eta_total:
            traci.vehicle.add(self.name, self.routeID, typeID=self.typeID, departSpeed="max")
            self.deployed = True
        # Check if the ego vehicle has passed the release point
        if self.deployed and ego_dist < self.release_point:
            self.done = True
        # If the conflict vehicle is en route but not past the release point, potentially update its speed
        if self.deployed and not self.done:
            conflict_dist = self.conflict_lane_length - traci.vehicle.getLanePosition(self.name)
            conflict_speed = traci.vehicle.getSpeed(self.name)
            if conflict_speed == 0:
                conflict_speed = 0.0001  # avoid division by zero
            conflict_eta = conflict_dist / conflict_speed
            if abs(conflict_eta - ego_eta) > 0.25:
                new_conflict_speed = conflict_dist / ego_eta
                if new_conflict_speed < 0:
                    print("Negative speed calculated. Setting to zero.")
                    new_conflict_speed = 0
                traci.vehicle.slowDown(self.name, new_conflict_speed, 0)

    def reset(self):
        """Resets this object by removing the vehicle from the simulation and allowing re-deployment."""
        traci.vehicle.remove(self.name, tc.REMOVE_VAPORIZED)
        self.deployed = False
        self.done = False


def read_target_points_from_file(file, type_value="target"):
    """
    Reads POIs from a Sumo XML additionals file and generates a dict of coordinates with IDs as keys.
    Only polygons with specified type are processed.
    :param file: path to Sumo XML additionals file
    :param type_value: Type string to match for polygons. Only polygons with this type are processed.
    :return: dict with IDs as keys and (x, y) coordinates as values
    :type file: str
    :type type_value: str
    """
    root = ET.parse(file).getroot()
    targets_points = dict()
    for child in root:
        attrs = child.attrib
        if child.tag == "poi" and "type" in attrs and attrs["type"] == type_value:
            x, y = float(attrs["x"]), float(attrs["y"])
            targets_points[attrs["id"]] = (x, y)
    return targets_points
