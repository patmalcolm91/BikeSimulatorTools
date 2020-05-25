"""
Module for using Sumo polygons as triggers when the ego vehicle enters or exits.
This is useful because the ego vehicle in the bike simulator is not detected by Sumo detectors.

Author: Patrick Malcolm
"""

import xml.etree.ElementTree as ET
import shapely.geometry
import traci


# Constants ============================================================================================================

class _Event:
    """Class used to define constants describing trigger events. Not intended for direct use."""
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return "< " + self.name.capitalize() + " Event >"

    def __eq__(self, other):
        return self.name == other.name


ENTRY = _Event("ENTRY")
EXIT = _Event("EXIT")
NO_CHANGE = _Event("NO_CHANGE")


# Classes and Functions ================================================================================================

class Trigger:
    def __init__(self, id, points):
        """
        Initializes a Trigger object
        :param id: the trigger id
        :param points: list of polygon points, either as an iterable of (x,y) tuples, or a string like "x1,y1 x2,y2 ..."
        :type id: str
        """
        self.id = id
        if type(points) == str:
            coords = [(float(x), float(y)) for x, y in [p.split(",") for p in points.split(" ")]]
        else:
            coords = points
        self.shape = shapely.geometry.Polygon(coords)
        self.state = False  # The trigger state as of the last check
        self.last_check = -1  # The simulation time at which the last check was performed
        self.last_event = NO_CHANGE  # The event triggered when the last check was performed

    def __repr__(self):
        point_string = " ".join([str(p[0]) + "," + str(p[1]) for p in self.shape.boundary.coords])
        return "Trigger('"+self.id + "', '" + point_string + "')"

    def includes(self, point):
        """
        Checks whether the trigger area includes the specified point.
        :param point: current coordinate of the ego vehicle
        :return: True if point contained within trigger area, else False
        :type point: (float, float)
        """
        return shapely.geometry.Point(point).within(self.shape)

    def check(self, point):
        """
        Evaluates whether the trigger area was entered or exited since the last call to this function.
        This function is safe against multiple calls per simulation timestep.
        :param point: current coordinate of the ego vehicle
        :return: ENTRY if vehicle just entered, EXIT if vehicle just exited, NO_CHANGE if no change since last call
        :type: (float, float)
        """
        sim_time = traci.simulation.getTime()
        if sim_time > self.last_check:
            old_state = self.state
            self.state = self.includes(point)
            self.last_check = sim_time
            if self.state != old_state:
                self.last_event = ENTRY if self.state is True else EXIT
            else:
                self.last_event = NO_CHANGE
        return self.last_event

    def check_entry(self, point):
        """
        Simplified version of check() which only evaluates whether the trigger area was entered.
        :param point: current coordinate of the ego vehicle
        :return: True if vehicle just entered, else False
        :type point: (float, float)
        """
        chk = self.check(point)
        return chk == ENTRY

    def check_exit(self, point):
        """
        Simplified version of check() which only evaluates whether the trigger area was exited.
        :param point: current coordinate of the ego vehicle
        :return: True if vehicle just exited, else False
        :type point: (float, float)
        """
        chk = self.check(point)
        return chk == EXIT


class Triggers:
    def __init__(self):
        self._trigger_list = []

    def append(self, trigger):
        """
        Append Trigger to Triggers list
        :param trigger: trigger to append
        :return: None
        :type trigger: Trigger
        """
        self._trigger_list.append(trigger)

    def __getitem__(self, item):
        """
        :rtype: Trigger
        """
        if type(item) == str:
            for trigger in self._trigger_list:
                if trigger.id == item:
                    return trigger
            raise IndexError("No trigger with id ", item)
        elif type(item) == int:
            return self._trigger_list[item]
        else:
            TypeError("Triggers can not be indexed with object of type '" + type(item).__name__ + "'")

    def __iter__(self):
        return iter(self._trigger_list)


def read_triggers_from_file(file, type_value="trigger"):
    """
    Reads polygons from a Sumo XML additionals file and generates a list of Trigger objects.
    Only polygons with specified type are processed.
    :param file: Sumo additionals file to read.
    :param type_value: Type string to match for polygons. Only polygons with this type are processed.
    :return: list of Trigger objects
    :type file: str
    :type type_value: str
    """
    root = ET.parse(file).getroot()
    triggers = Triggers()
    for child in root:
        attrs = child.attrib
        if child.tag == "poly" and "type" in attrs and attrs["type"] == type_value:
            triggers.append(Trigger(attrs["id"], attrs["shape"]))
    return triggers


def check_triggers(triggers, point):
    """
    Checks a list of triggers and returns a dict containing the results
    :param triggers: list of Trigger objects to check
    :param point: ego vehicle coordinate to check against the triggers
    :return: dict of trigger IDs and events
    :type triggers: list[Trigger]
    :type point: (float, float)
    """
    return {trigger.id: trigger.check(point) for trigger in triggers}


def check_triggers_state(triggers, point):
    """
    Checks a list of triggers and returns a dict containing the results of their states (not events)
    :param triggers: list of Trigger objects to check
    :param point: ego vehicle coordinate to check against the triggers
    :return: dict of trigger IDs and states (True/False)
    :type triggers: list[Trigger]
    :type point: (float, float)
    """
    return {trigger.id: trigger.includes(point) for trigger in triggers}
