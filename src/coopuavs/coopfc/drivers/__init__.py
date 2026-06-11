"""Sensor drivers: HAL port -> SI conversion -> topic publish.

One driver class per device, each registered as a scheduler task at its
device's rate. Every driver shares the same staleness contract
(_base.Driver): a tick that finds no new HAL seq counts one stale tick;
`stale` latches true at `stale_after` consecutive misses and clears on
the next fresh frame (the P5 CBIT *_STALE monitors read these).
"""
