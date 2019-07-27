"""
Module for using Sumo polygons as triggers when the ego vehicle enters or exits.
This is useful because the ego vehicle in the bike simulator is not detected by Sumo detectors.

Author: Patrick Malcolm
"""

import xml.etree.ElementTree as ET
import shapely.geometry


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
        self.state = False

    def __repr__(self):
        point_string = " ".join([str(p[0]) + "," + str(p[1]) for p in self.shape.boundary.coords])
        return "Trigger('"+self.id + "', '" + point_string + "')"

    def check(self, point):
        """
        Evaluates whether the trigger area was entered or exited since the last call to this function.
        :param point: current coordinate of the ego vehicle
        :return: ENTRY if vehicle just entered, EXIT if vehicle just exited, NO_CHANGE if no change since last call
        :type: tuple(float)
        """
        old_state = self.state
        self.state = shapely.geometry.Point(point).within(self.shape)
        if self.state != old_state:
            return ENTRY if self.state is True else EXIT
        else:
            return NO_CHANGE

    def check_entry(self, point):
        """
        Simplified version of check() which only evaluates whether the trigger area was entered.
        :param point: current coordinate of the ego vehicle
        :return: True if vehicle just entered, else False
        :type point: tuple[float]
        """
        chk = self.check(point)
        return chk == ENTRY

    def check_exit(self, point):
        """
        Simplified version of check() which only evaluates whether the trigger area was exited.
        :param point: current coordinate of the ego vehicle
        :return: True if vehicle just exited, else False
        :type point: tuple[float]
        """
        chk = self.check(point)
        return chk == EXIT


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
    triggers = []
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
    :type point: tuple(float)
    """
    return {trigger.id: trigger.check(point) for trigger in triggers}


# Test Code ============================================================================================================

if __name__ == "__main__":
    print("Running test code...")
    triggers = read_triggers_from_file("../triggers.xml")
    print(check_triggers(triggers, (40, 112)))
    print(check_triggers(triggers, (60, 112)))
    print(check_triggers(triggers, (60, 112)))
    print(check_triggers(triggers, (70, 112)))
