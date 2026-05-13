# Docker Image: Unicore N4 ROS2 Driver

This directory contains the ROS 2 C++ driver for Unicore N4 GNSS receivers packaged as a Docker image.

## Build

```bash
docker build -t unicore_gnss:latest .
```

### Multi-architecture build

Build both amd64 and arm64 images and push to GitHub Container Registry:

```bash
docker buildx build --platform linux/amd64,linux/arm64 \
  -t ghcr.io/<your-org>/unicore-ros2:latest \
  --push .
```

You must be logged in to GHCR first:

```bash
echo "$GHCR_PAT" | docker login ghcr.io -u <your-gh-user> --password-stdin
```

For convenience, use `./build.sh`.

If you want local images for each architecture separately:

```bash
docker buildx build --platform linux/amd64 -t unicore_gnss:amd64 --load .
```

```bash
docker buildx build --platform linux/arm64 -t unicore_gnss:arm64 --load .
```

For convenience, use `./build.sh`.

## Run

Mount the serial device where your Unicore receiver is connected (default `/dev/ttyUSB0`):

```bash
docker run --rm -it \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \
  --network host \
  unicore_gnss:latest
```

## Configuration

The driver reads parameters from `/opt/unicore_gnss/share/unicore_gnss/config/unicore.yaml` inside the container.

To override at runtime, pass ROS 2 parameter arguments:

```bash
docker run --rm -it \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \
  --network host \
  unicore_gnss:latest \
  ros2 run unicore_gnss unicore_node \
    --ros-args \
    -p port:=/dev/ttyUSB0 \
    -p baudrate:=921600 \
    -p frame_id:=gnss
```

## Docker Compose Integration

Example in a larger ROS 2 system (e.g., `docker-compose.yml`):

```yaml
services:
  gnss:
    image: unicore_gnss:latest
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
    network_mode: host
    restart: unless-stopped
```

## Output Topics

- `/gnss/fix` — `sensor_msgs/NavSatFix`
- `/gnss/azimuth` — `compass_interfaces/Azimuth`
- `/gnss/diagnostics` — `diagnostic_msgs/DiagnosticArray`

## Serial Port Configuration

The receiver **must** be configured to output NMEA/Unicore sentences on its serial port:

```
config com2 921600
PVTSLNA com2 0.05
GPHPR com2 0.05
BESTNAVA com2 0.05
```

(Adjust COM port and frequency as needed for your setup.)
