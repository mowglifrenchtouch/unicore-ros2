# unicore_gnss

Unicore N4 ROS2 Driver.

Advanced ROS2 GNSS/RTK backend for Unicore N4 receivers.

This repository is now organized as a clean ROS 2 package with the driver implementation in `src/`, public headers in `include/unicore_gnss/`, and runtime configuration in `config/unicore.yaml`.

## Build

```bash
colcon build --packages-select unicore_gnss
```

## Run

```bash
ros2 launch unicore_gnss unicore_launch.py
```

## Configuration

The default runtime parameters are in `config/unicore.yaml`.

Compatibility aliases are kept for existing integrations:
- `um982_launch.py`
- `um982.yaml`
- `um982_node`
- `tools/um982_live_validate.py`

## Independent ROS 2 implementation

This ROS 2 C++ package is an independent reimplementation of Unicore N4 GNSS protocol handling.
It does not contain source code from the original Python driver.
Protocol parsing was implemented from public Unicore / NMEA / N4 documentation.

Support for additional Unicore sentences such as `KSXT` is intended to be added as a protocol-driven extension, not by porting legacy driver code.
