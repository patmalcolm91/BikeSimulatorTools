"""
Module for creating higher-level configurations to manage interactions between other modules in this package.

Author: Patrick Malcolm
"""
from typing import List, Any

from . import DynamicFlows
from . import Triggers


class _TriggeredFlow:
    def __init__(self, flow, enable_trigger_id, disable_trigger_id, enable_event, disable_event):
        """
        Initializes a _TriggeredFlow object. Only intended for internal use.
        :param flow: DynamicFlow object to enable/disable based on the trigger events
        :param enable_trigger_id: id of the enabling Trigger object
        :param disable_trigger_id: id of the disabling Trigger object
        :param enable_event: the event to detect on the enabling Trigger
        :param disable_event: the event to detect on the disabling Trigger
        :type flow: DynamicFlows.DynamicFlow
        :type enable_trigger_id: str
        :type disable_trigger_id: str
        :type enable_event: Triggers._Event
        :type disable_event: Triggers._Event
        """
        self.flow = flow
        self.enable_trigger_id = enable_trigger_id
        self.disable_trigger_id = disable_trigger_id
        self.enable_event = enable_event
        self.disable_event = disable_event


class TriggeredFlows:
    """
    Class for connecting Trigger state changes to DynamicFlow enable and disable events.
    Checks all triggers and enables/disables flows when the corresponding Trigger object registers the specified event.
    """
    def __init__(self, triggers):
        """
        Initializes a TriggeredFlows configuration object.
        :param triggers: list of all triggers in the simulation
        :type triggers: list[Triggers.Trigger]
        """
        self.triggers = triggers
        self.flows = []  # type: list[_TriggeredFlow]

    def add(self, flow, enable_trigger_id, disable_trigger_id, enable_event=Triggers.ENTRY, disable_event=Triggers.ENTRY):
        """
        Adds a new DynamicFlow to the configuration with the specified triggers.
        :param flow: DynamicFlow object to be enabled/disabled by Triggers
        :param enable_trigger_id: id of enabling Trigger
        :param disable_trigger_id: id of disabling Trigger
        :param enable_event: enabling event
        :param disable_event: disabling event
        :return: None
        :type flow: DynamicFlows.DynamicFlow
        :type enable_trigger_id: str
        :type disable_trigger_id: str
        :type enable_event: Triggers._Event
        :type disable_event: Triggers._Event
        """
        tf = _TriggeredFlow(flow, enable_trigger_id, disable_trigger_id, enable_event, disable_event)
        self.flows.append(tf)

    def run(self, point):
        """
        Checks Triggers against the specified point coordinates and toggles and runs the corresponding DynamicFlows.
        :param point: coordinate of ego vehicle to check against Triggers or None to skip trigger checks
        :return: None
        :type point: (float, float)
        """
        if point is not None:
            trigger_states = Triggers.check_triggers(self.triggers, point)
            for tf in self.flows:
                if tf.enable_trigger_id in trigger_states and trigger_states[tf.enable_trigger_id] == tf.enable_event:
                    tf.flow.enable()
                if tf.disable_trigger_id in trigger_states and trigger_states[tf.disable_trigger_id] == tf.disable_event:
                    tf.flow.disable()

        for tf in self.flows:
            tf.flow.run()
