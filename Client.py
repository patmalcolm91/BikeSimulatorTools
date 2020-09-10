"""
Contains a basic TraCI client class that can be run in either master or slave mode.

Author: Patrick Malcolm
"""

import traci


class Client:
    def __init__(self, sumo_bin="sumo-gui", traci_ip="127.0.0.1", traci_port=8813, connection_order=1,
                 connection_label="BikeSimulatorTools"):
        self.sumo_bin = sumo_bin
        self.traci_ip = traci_ip
        self.traci_port = traci_port
        self.connection_order = connection_order
        self.connection_label = connection_label
        self.running = False
        self.time = None
        self._cxn = None
        self._mode = None
        self._update_actions = []  # list of (function, args, kwargs) tuples to call every simulation step

    def run_master(self, sumo_cfg, ego_route=None, ego_start_time=1, ego_vehID="ego", ego_kwargs=None):
        assert self.running is False, "Client already running!"
        traci.start([self.sumo_bin, "-c", sumo_cfg])
        self.running = True
        self._mode = "master"
        if ego_route is not None:
            ego_kwargs = ego_kwargs if ego_kwargs is not None else dict()
            self.add_update_action(lambda: traci.vehicle.add(ego_vehID, ego_route, **ego_kwargs) if self.time == ego_start_time else None)

    def run_slave(self):
        assert self.running is False, "Client already running!"
        traci.init(port=self.traci_port, label=self.connection_label)
        self._cxn = traci.getConnection(self.connection_label)
        self._cxn.setOrder(self.connection_order)
        self.running = True
        self._mode = "slave"

    def add_update_action(self, fun, *args, **kwargs):
        self._update_actions.append((fun, args, kwargs))

    def update(self):
        traci.simulationStep()
        self.time = traci.simulation.getTime()
        if self._mode == "slave":
            # TODO: implement ego trajectory playback
            pass
        for fun, args, kwargs in self._update_actions:
            fun(*args, **kwargs)

    def __del__(self):
        if self._cxn is not None:
            self._cxn.close()

