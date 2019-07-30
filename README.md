# BikeSimulatorTools
A Python library with miscellaneous tools for use in a SUMO and DYNA4 bicycle simulation environment.

## Modules

### Triggers
Module for using Sumo polygons as triggers when the ego vehicle enters or exits.
This is useful because the ego vehicle in the bike simulator is not detected by Sumo detectors.

### DataSync
Contains classes and functions to easily send and receive values via UDP.
Any value or list of values which can be packed by Python's struct module can be sent.
For information on packing format strings, see https://docs.python.org/3/library/struct.html#format-characters

### DynamicFlows
Contains classes and functions for creating and modifying flows in a SUMO simulation dynamically.
The class also allows a probabilistic mix of different vehicle types.

### ConflictVehicles
Module for inserting a single vehicle timed to create a conflict with the ego vehicle at an intersection.
The vehicle will be sent at a time which will bring it to the end of the first lane in its route (+/- an offset)
when the ego vehicle is projected to reach a specified target point.

### SimConfig
Module for creating higher-level configurations to manage interactions between other modules in this package.

#### TriggeredFlows
Class for connecting Trigger state changes to DynamicFlow enable and disable events.
Checks all triggers and enables/disables flows when the corresponding Trigger object registers the specified event.