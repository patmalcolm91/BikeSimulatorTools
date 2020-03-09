"""
Contains classes and functions for creating and modifying flows in a SUMO simulation dynamically.
The class also allows a probabilistic mix of different vehicle types.

Author: Patrick Malcolm
"""

import traci
import traci.constants as tc
import random
import numpy as np
from . import RouteTools


class DynamicFlow:
    def __init__(self, origin, destination, probability=None, vehicleMix=None, departSpeed="max", arrivalSpeed="current",
                 via=None, name=None, enabled=True, departPos="0", vaporizeOnDisable=False, maxCount=None,
                 departAnywhere=False, headway=None):
        """
        Initializes a DynamicFlow object. Stochastic if probability given, deterministic if headway given.
        :param origin: the 'from' edge for the route
        :param destination: the 'to' edge for the route
        :param probability: probability for emitting a vehicle each second
        :param vehicleMix: dict with vehicle type IDs as keys and probabilities as values. Defaults to {"passenger": 1}
        :param departSpeed: departSpeed for route (see SUMO documentation)
        :param arrivalSpeed: arrivalSpeed for route (see SUMO documentation)
        :param name: the name to use for the route. Defaults to "origin-destination"
        :param enabled: whether the flow is enabled or disabled
        :param departPos: determines position on lane at which the vehicle is tried to be inserted. See Sumo docs.
        :param vaporizeOnDisable: if True, vehicles will be vaporized upon disabling the flow
        :param maxCount: maximum number of vehicles to generate
        :param departAnywhere: if True, vehicles may depart from any edge along the route, rather than just the first. Only supported for pedestrians.
        :param headway: desired headway of generated vehicles (for deterministic flows). If set, one vehicle will be generated each {headway} seconds
        :type origin, destination: str
        :type probability: float
        :type vehicleMix: dict
        :type departSpeed, arrivalSpeed, name: str
        :type enabled: bool
        :type vaporizeOnDisable: bool
        :type maxCount: int
        :type departAnywhere: bool
        :type headway: float
        """
        self.origin = origin
        self.destination = destination
        self.probability = probability
        self.headway = headway
        if probability is not None and headway is not None:
            raise UserWarning("Both probability and headway defined in DynamicFlow, but only one should be defined.")
        if probability is None and headway is None:
            raise UserWarning("Must define either probability or headway in DynamicFlow.")
        self.steps_since_last_vehicle = np.inf
        self.vehicleMix = vehicleMix if vehicleMix is not None else {"passenger": 1}
        vClasses = set([traci.vehicletype.getVehicleClass(vType) for vType in self.vehicleMix])
        if "pedestrian" in vClasses:
            self._pedestrianFlag = True
            if len(vClasses) > 1:
                raise ValueError("Pedestrian and vehicular flows can't be mixed in a DynamicFlow object.")
        else:
            self._pedestrianFlag = False
        self.edges = dict()  # dict to hold list of route edges for each vType in the flow
        self.lanes = dict()  # dict to hold list of route lanes for each vClass in the flow
        for vType in self.vehicleMix:
            vClass = traci.vehicletype.getVehicleClass(vType)
            self.edges[vType] = traci.simulation.findRoute(self.origin, self.destination, vType).edges
            self.lanes[vClass] = [RouteTools.get_rightmost_allowed_lane(edge, vClass) for edge in self.edges[vType]]
        self.departSpeed = departSpeed
        self.arrivalSpeed = arrivalSpeed
        self.via = via if via is not None else ""
        self.name = name if name is not None else origin + "-" + destination
        traci.route.add(self.name, [origin, destination])
        traci.route.setParameter(self.name, "via", self.via)
        self.enabled = enabled
        self.departPos = departPos
        self.vaporizeOnDisable = vaporizeOnDisable
        self.maxCount = np.inf if maxCount is None else maxCount
        self.departAnywhere = departAnywhere
        self.count = 0

    def enable(self):
        """Enables this dynamic flow."""
        self.enabled = True

    def disable(self):
        """Disables this dynamic flow."""
        self.enabled = False
        if self.vaporizeOnDisable:
            self.vaporize()

    def run(self):
        """Processes the dynamic flow and inserts a vehicle if necessary. Should be run every simulation step."""
        time_step = traci.simulation.getDeltaT()
        self.steps_since_last_vehicle += 1
        if self.headway is not None:  # if deterministic flow
            send_vehicle = time_step * self.steps_since_last_vehicle >= self.headway
        else:  # if stochastic flow
            p = 1 - (1 - self.probability)**time_step  # Get probability for sim step
            send_vehicle = random.random() < p
        if self.enabled and send_vehicle and self.count < self.maxCount:
            self.steps_since_last_vehicle = 0
            vTypes = np.array(list(self.vehicleMix.keys()))
            pdf = np.array([self.vehicleMix[v] for v in vTypes], dtype=float)
            pdf /= sum(pdf)
            vType = np.random.choice(vTypes, p=pdf)
            vClass = traci.vehicletype.getVehicleClass(vType)
            if self._pedestrianFlag:
                pedID = self.name+".ped."+str(self.count)
                edges = self.edges[vType]
                lanes = self.lanes[vClass]
                if self.departPos == tc.DEPARTFLAG_POS_RANDOM:
                    if self.departAnywhere:
                        departLane, departPos = RouteTools.random_depart_pos(lanes)
                    else:
                        departLane, departPos = lanes[0], RouteTools.random_depart_pos(lanes[0])
                else:
                    departLane = lanes[0]
                    departPos = self.departPos
                departEdge = traci.lane.getEdgeID(departLane)
                traci.person.add(pedID, departEdge, departPos, typeID=vType)
                edges = edges[edges.index(departEdge):]
                traci.person.appendWalkingStage(pedID, edges, tc.ARRIVALFLAG_POS_MAX)
            else:
                traci.vehicle.add(self.name+"."+str(self.count), self.name, typeID=vType, departSpeed=self.departSpeed,
                                  arrivalSpeed=self.arrivalSpeed, departPos=self.departPos)
            self.count += 1

    def vaporize(self):
        """Vaporizes all vehicles belonging to this flow."""
        if self._pedestrianFlag is True:
            ped_list = traci.person.getIDList()
            for i in range(self.count):
                pName = self.name+".ped."+str(i)
                if pName in ped_list:
                    traci.person.removeStages(pName)
        else:
            veh_list = traci.vehicle.getIDList()
            for i in range(self.count):
                vName = self.name+"."+str(i)
                if vName in veh_list:
                    traci.vehicle.remove(vName)
