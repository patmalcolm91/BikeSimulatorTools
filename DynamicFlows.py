"""
Contains classes and functions for creating and modifying flows in a SUMO simulation dynamically.
The class also allows a probabilistic mix of different vehicle types.

Author: Patrick Malcolm
"""

import traci
import random
import numpy as np


class DynamicFlow:
    def __init__(self, origin, destination, probability, vehicleMix=None, departSpeed="max", arrivalSpeed="current",
                 via=None, name=None, enabled=True, departPos="0", vaporizeOnDisable=False):
        """
        Initializes a DynamicFlow object.
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
        :type origin, destination: str
        :type probability: float
        :type vehicleMix: dict
        :type departSpeed, arrivalSpeed, name: str
        :type enabled: bool
        :type departPos: str
        :type vaporizeOnDisable: bool
        """
        self.origin = origin
        self.destination = destination
        self.probability = probability
        self.vehicleMix = vehicleMix if vehicleMix is not None else {"passenger": 1}
        self.departSpeed = departSpeed
        self.arrivalSpeed = arrivalSpeed
        self.via = via if via is not None else ""
        self.name = name if name is not None else origin + "-" + destination
        traci.route.add(self.name, [origin, destination])
        traci.route.setParameter(self.name, "via", self.via)
        self.enabled = enabled
        self.departPos = departPos
        self.vaporizeOnDisable = vaporizeOnDisable
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
        p = 1 - (1 - self.probability)**traci.simulation.getDeltaT()  # Get probability for sim step
        if self.enabled and random.random() < p:
            vTypes = np.array(list(self.vehicleMix.keys()))
            pdf = np.array([self.vehicleMix[v] for v in vTypes], dtype=float)
            pdf /= sum(pdf)
            vType = np.random.choice(vTypes, p=pdf)
            traci.vehicle.add(self.name+"."+str(self.count), self.name, typeID=vType, departSpeed=self.departSpeed,
                              arrivalSpeed=self.arrivalSpeed, departPos=self.departPos)
            self.count += 1

    def vaporize(self):
        """Vaporizes all vehicles belonging to this flow."""
        veh_list = traci.vehicle.getIDList()
        for i in range(self.count):
            vName = self.name+"."+str(i)
            if vName in veh_list:
                traci.vehicle.remove(vName)
