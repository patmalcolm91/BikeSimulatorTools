"""
Contains miscellaneous tools for dealing with routes, departures, and arrivals.
A function for replicating and extending random departure position functionality for people is included
(currently this can only be done in TraCI with vehicles, and not people.
"""

import traci
import numpy as np


def random_depart_pos(lanes):
    """
    Returns a random depart position on the specified lane or lanes. If a single lane id is given, only a position
    is returned. If multiple lanes are given, a tuple with the randomly selected lane and the position is returned.
    :param lanes:
    :return: position along lane if a single lane given, (lane, position) tuple if multiple lanes given
    """
    if type(lanes) in [list, tuple]:
        # the extended case where multiple lanes are given
        lengths = [traci.lane.getLength(lane) for lane in lanes]  # get list of all lane lengths
        pos = np.random.random()*sum(lengths)  # randomly choose a position along the path formed by the lanes
        # find the lane and local lane position corresponding to the randomly chosen position
        ipos = 0
        for i in range(len(lanes)):
            if pos < ipos + lengths[i]:
                return lanes[i], pos-ipos
            ipos += lengths[i]
    else:
        # the simple case where only one lane is given
        return np.random.random() * traci.lane.getLength(lanes)


def get_rightmost_allowed_lane(edge, vClass):
    """
    Returns the lane id of the rightmost lane on the given edge which allows the given vehicle class.
    Note: because of a limitation in TraCI, default generated lane names must not be changed or this function will break
    :param edge: edge from which to find a lane
    :param vClass: vehicle class
    :return: lane id of the rightmost lane allowed for vClass
    """
    candidates = []
    for lane in traci.lane.getIDList():
        if traci.lane.getEdgeID(lane) != edge:
            continue
        allowed = traci.lane.getAllowed(lane)
        if len(allowed) == 0 or vClass in allowed:
            candidates.append(lane)
    if len(candidates) == 0:
        raise traci.TraCIException("No allowed lane for vClass "+vClass+" on edge "+edge+".")
    return np.sort(candidates)[0]  # choose the first alphabetically, which should be the rightmost
