# mowgli_unicore_gnss

ROS 2 C++ driver for Unicore UM982 GNSS receivers.

This repository is now organized as a clean ROS 2 package with the driver implementation in `src/`, public headers in `include/mowgli_unicore_gnss/`, and runtime configuration in `config/um982.yaml`.

## Build

```bash
colcon build --packages-select mowgli_unicore_gnss
```

## Run

```bash
ros2 launch mowgli_unicore_gnss um982_launch.py
```

## Configuration

The default runtime parameters are in `config/um982.yaml`.

## Independent ROS 2 implementation

This ROS 2 C++ package is an independent reimplementation of UM982 GNSS protocol handling.
It does not contain source code from the original Python driver.
Protocol parsing was implemented from public UM982 / NMEA / Unicore documentation.

Support for additional Unicore sentences such as `KSXT` is intended to be added as a protocol-driven extension, not by porting legacy driver code.
