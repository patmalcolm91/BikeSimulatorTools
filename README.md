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
