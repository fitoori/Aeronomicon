# Aeronomicon

A collection of scripts and utilities to streamline working with Intel RealSense sensors, MAVLink, and ancillary tooling for aerial robotics projects. Part of the Wireless Autonomous Terrestrial Navigation Experiment (W.A.T.N.E.) - an ongoing effort to design an Unmanned Aerial Vehicle (UAV) that is capable of independently and precisely navigating through both urban and rural terrestrial environments.

## Functioning Contents

- **onics-t.py** - Re-rework of Thien94's `precland` script which used Intel's RealSense T265 Tracking Camera (discontinued) to achieve GPS-less precision loiter capabilities and AprilTag-based self-landing.
- **apriltags3.py** – Upstream apriltags dependency preserved for compatibility and slightly enhanced/modernized.
- **uplink.sh** – Bash script to manage an LTE uplink with self-recovery logic and a daily reboot at 01:00.
- **mavproxy.service** – Systemd unit file for running MAVProxy as a managed service.

## Diagnostic Tools & Utilities

### RealSense

- **util/tcap.py** - Capture a single frame from the T265.
- **util/dcap.py** - Capture a single frame from D4XX-series depth cameras.

### LTE

- **util/uplink-diagnostic.sh** - Run LTE uplink diagnostics and report link status.
- **util/lte-signal-strength.sh** - Poll and log LTE signal strength.

### System Utilities

- **util/hall-monitor.sh** - Watch a single service and restart it on crash.
- **util/logfix.py** - Post-process `uplink.sh` logs into condensed, one-line-per-month reports.
- **util/usb-power-cycle.sh** - Power-cycle USB devices for recovery.
- **util/webapp-watchdog.sh** - Monitor the webapp service and restart if needed.

## Webapp & Services

- **webapp/** – Frontend assets for the local status/monitoring UI.
- **webapp.service** – Systemd unit for the webapp.
- **webapp-watchdog.service** – Systemd unit for the webapp watchdog.
- **webapp-watchdog.timer** – Timer to run the watchdog on a schedule.

## Legacy Repository Contents

- **legacy/ONICS2.py** – Re-rework of Thien94's `rs_to_mavlink.py` script with multicore support.
- **legacy/ONICS.py** – Complete rework of Thien94's script that adds logging and thread-management options.
- **legacy/uplink.sh** – Bash script to manage an LTE uplink with self-recovery logic and a daily reboot at 01:00.
- **legacy/logfix.py** – Post-processes `uplink.sh` logs into condensed, one-line-per-month reports.
- **legacy/install/** – Install helpers for dependencies, services, Intel RealSense, and ROS (including `install-deps.sh`, `install-services.sh`, `install_realsense.sh`, and `install_ros.sh`). _Currently untested._
- **legacy/hall-monitor.sh** – Watches a single service and restarts it if it crashes.
- **legacy/t265_precland_apriltag.py** – Optimized fork of Thien94's script of the same name. _Currently untested._
- **legacy/d4xx_to_mavlink.py** – Optimized fork of Thien94's script of the same name. _Currently untested._
- **legacy/d435i_to_mavlink.py** – Optimized fork of Thien94's script with added support for the D435i IMU. _Currently untested._

## Additional Notes

- Your Mileage May Vary.
- Untested scripts may require tweaks or additional dependencies—use caution in production environments.
- When modifying systemd units or shell scripts, keep backups of working configurations so you can quickly revert.
- A lot of the hardware (and as a result software) is outdated/discontinued and therefore not worth pursuing going forward. If you're actually trying to build one of your own, please don't bother. Instead, reach out to me! 
