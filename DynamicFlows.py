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
                 via=None, name=None, enabled=True):
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
        :type origin, destination: str
        :type probability: float
        :type vehicleMix: dict
        :type departSpeed, arrivalSpeed, name: str
        :type enabled: bool
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
        self.count = 0

    def enable(self):
        """Enables this dynamic flow."""
        self.enabled = True

    def disable(self):
        """Disables this dynamic flow."""
        self.enabled = False

    def run(self):
        """Processes the dynamic flow and inserts a vehicle if necessary. Should be run every simulation step."""
        if self.enabled and random.random() < self.probability * traci.simulation.getDeltaT():
            vTypes = np.array(list(self.vehicleMix.keys()))
            pdf = np.array([self.vehicleMix[v] for v in vTypes], dtype=float)
            print(pdf)
            pdf /= sum(pdf)
            vType = np.random.choice(vTypes, p=pdf)
            traci.vehicle.add(self.name+"."+str(self.count), self.name, typeID=vType, departSpeed=self.departSpeed,
                              arrivalSpeed=self.arrivalSpeed)
            self.count += 1
