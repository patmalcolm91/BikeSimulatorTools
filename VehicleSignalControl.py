"""
Contains tools for controlling vehicle signals.
"""

import traci
import numpy as np
from .Triggers import EXIT, ENTRY

BLINK_DEFAULT = [0.4, 0.4]


class Signal:
    def __init__(self, bit, pattern=None, enabled=False):
        """
        Initialize a controllable vehicle signal.

        :param bit: the bit corresponding to the vehicle signal (see SUMO documentation)
        :param pattern: list of on/off durations for blinking pattern. None if light should not blink.
        :param enabled: whether the signal is enabled by default
        """
        self.bit = bit
        self.pattern = pattern if pattern is not None else [np.inf, 0]
        self._enabled = enabled
        self._pattern_i = 0
        self._last_changed = 0
        self.state = enabled

    def enable(self):
        if self._enabled is True:
            return
        self._enabled = True
        self._pattern_i = 0
        self._last_changed = traci.simulation.getTime()

    def disable(self):
        self._enabled = False

    def update(self):
        if not self._enabled:
            self.state = False
            return
        t = traci.simulation.getTime()
        if t - self._last_changed >= self.pattern[self._pattern_i]:
            self._pattern_i = (self._pattern_i+1) % len(self.pattern)
            self._last_changed = t
        self.state = (self._pattern_i+1) % 2  # even index -> light on, odd index -> light off


class SignallingVehicle:
    def __init__(self, vehID):
        """
        Initialize a SignallingVehicle object, which can be used to control a vehicle's signals either manually or
        via Triggers.
        :param vehID: Sumo vehicle ID of vehicle whose signals are to be controlled
        """
        self.id = vehID
        self.signals = []
        self.enable_triggers = []
        self.disable_triggers = []
        self.enable_events = []
        self.disable_events = []

    def add_controlled_signal(self, signal, enable_trigger=None, disable_trigger=None, enable_event=ENTRY,
                              disable_event=ENTRY):
        """
        Add a controllable Signal to the object.
        :param signal: Signal object
        :param enable_trigger: Trigger object to use for enabling signal
        :param disable_trigger: Trigger object to use for disabling signal
        :param enable_event: Event to use for enabling signal (default: Triggers.ENTRY)
        :param disable_event: Event to use for disabling signal (default: Triggers.ENTRY)
        :return: None
        """
        self.signals.append(signal)
        self.enable_triggers.append(enable_trigger)
        self.enable_events.append(enable_event)
        self.disable_triggers.append(disable_trigger)
        self.disable_events.append(disable_event)

    def update(self):
        """
        Check triggers and update signal state in Sumo as necessary. Must be called every simulation step.
        :return: None
        """
        if self.id not in traci.vehicle.getIDList():
            return
        traci.vehicle.setSignals(self.id, -1)
        current_signals = traci.vehicle.getSignals(self.id)
        for i, signal in enumerate(self.signals):
            pos = traci.vehicle.getPosition(self.id)
            if self.enable_triggers[i] is not None and self.enable_triggers[i].check(pos) == self.enable_events[i]:
                signal.enable()
            if self.disable_triggers[i] is not None and self.disable_triggers[i].check(pos) == self.disable_events[i]:
                signal.disable()
            signal.update()
            if signal.state:
                current_signals = current_signals | (1 << signal.bit)
            else:
                current_signals = current_signals & ~(1 << signal.bit)
        traci.vehicle.setSignals(self.id, current_signals)

