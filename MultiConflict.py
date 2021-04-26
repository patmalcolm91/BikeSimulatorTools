"""
A module for configuring a conflict vehicle to meet an ego vehicle multiple times along their respective routes.
"""

import numpy as np
import traci
from shapely.geometry import LineString, Point
from typing import NamedTuple


class _ConflictTarget(NamedTuple):
    ego_station: float
    conflict_vehicle_station: float
    release_point: float


class _WaitTarget(_ConflictTarget):
    pass


class MultiConflict:
    """A class for coordinating two vehicles to arrive simultaneously at set points along their respective routes."""
    def __init__(self, ego_id, conflict_vehicle_id, ego_route=None, conflict_vehicle_route=None, release_point=10):
        self.ego_id = ego_id
        self.conflict_vehicle_id = conflict_vehicle_id
        self.ego_route = ego_route if ego_route is not None else traci.vehicle.getRouteID(self.ego_id)
        self.conflict_vehicle_route = conflict_vehicle_route if conflict_vehicle_route is not None \
            else traci.vehicle.getRouteID(self.conflict_vehicle_id)
        self._ego_trajectory = self._line_from_route(self.ego_route)
        self._conflict_vehicle_trajectory = self._line_from_route(self.conflict_vehicle_route)
        self._targets = []  # type: list[_ConflictTarget] # ego station, cv station, release point
        self.release_point = release_point
        self._active = False
        self._waiting = False

    @staticmethod
    def _line_from_route(route_id):
        """Convert a SUMO route to a Shapely LineString."""
        edges = traci.route.getEdges(route_id)
        coords = []
        for edge in edges:
            coords += traci.lane.getShape(edge+"_0")
        return LineString(coords)

    def _add_generic_target(self, target_class, ego_coords, conflict_vehicle_coords, release_point=None):
        release_point = release_point if release_point is not None else self.release_point
        _ep, _cvp = Point(ego_coords), Point(conflict_vehicle_coords)
        _e_station = self._ego_trajectory.project(_ep)
        _cv_station = self._conflict_vehicle_trajectory.project(_cvp)
        self._targets.append(target_class(_e_station, _cv_station, release_point))
        self._active = True

    def add_target(self, ego_coords, conflict_vehicle_coords, release_point=None):
        """Add a target to be managed."""
        self._add_generic_target(_ConflictTarget, ego_coords, conflict_vehicle_coords, release_point)

    def add_wait_target(self, ego_coords, conflict_vehicle_coords, braking_distance=None):
        """Add a wait target to be managed."""
        self._add_generic_target(_WaitTarget, ego_coords, conflict_vehicle_coords, release_point=braking_distance)

    def _control_wait_target(self):
        ego_target, cv_target, release_point = self._targets[0]
        ego_pos = traci.vehicle.getPosition(self.ego_id)
        ego_station = self._ego_trajectory.project(Point(ego_pos))
        cv_pos = traci.vehicle.getPosition(self.conflict_vehicle_id)
        cv_station = self._conflict_vehicle_trajectory.project(Point(cv_pos))
        if ego_station > ego_target:
            self._targets.pop(0)
            traci.vehicle.setSpeed(self.conflict_vehicle_id, -1)
            self._waiting = False
        elif cv_station < cv_target - release_point:
            traci.vehicle.setSpeed(self.conflict_vehicle_id, -1)
            self._waiting = False
        else:
            cv_speed = traci.vehicle.getSpeed(self.conflict_vehicle_id)
            dur = (cv_target - cv_station) / cv_speed / 2
            traci.vehicle.slowDown(self.conflict_vehicle_id, 0, dur)
            self._waiting = True

    def _control_conflict_target(self):
        ego_target, cv_target, release_point = self._targets[0]
        ego_speed = traci.vehicle.getSpeed(self.ego_id)
        if ego_speed == 0:
            ego_speed = 0.0001
        ego_pos = traci.vehicle.getPosition(self.ego_id)
        ego_station = self._ego_trajectory.project(Point(ego_pos))
        if ego_target - ego_station > release_point:
            ego_eta = (ego_target - ego_station) / ego_speed
            cv_pos = traci.vehicle.getPosition(self.conflict_vehicle_id)
            cv_station = self._conflict_vehicle_trajectory.project(Point(cv_pos))
            cv_remaining_dist = cv_target - cv_station
            cv_desired_speed = cv_remaining_dist / ego_eta
            traci.vehicle.setSpeed(self.conflict_vehicle_id, cv_desired_speed)
        else:
            self._targets.pop(0)

    def check(self):
        """Check vehicle trajectories and adjust conflict vehicle speed as necessary."""
        if len(self._targets) == 0:
            if self._active:
                self._active = False
                traci.vehicle.setSpeed(self.conflict_vehicle_id, -1)
            return None
        if type(self._targets[0]) == _ConflictTarget:
            self._control_conflict_target()
        elif type(self._targets[0]) == _WaitTarget:
            self._control_wait_target()
        else:
            raise NotImplementedError("Unsupported MultiConflict target type.")
