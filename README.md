# Aeronomicon

A collection of scripts and utilities to streamline working with Intel RealSense sensors, MAVLink, and ancillary tooling for aerial robotics projects. Part of the Wireless Autonomous Terrestrial Navigation Experiment (W.A.T.N.E.) - an ongoing effort to design an Unmanned Aerial Vehicle (UAV) that is capable of independently and precisely navigating through both urban and rural terrestrial environments. 

## Functioning Contents

- **onics-t.py** -  Re-rework of Thien94's `precland` script which used Intel's RealSense T265 Tracking Camera (discontinued) to achieve GPS-less precision loiter capabilities and AprilTag-based self-landing.
- **apriltags3.py** – Upstream apriltags dependency preserved for compatibility and slightly enhanced/modernized.
- **uplink.sh** – Bash script to manage an LTE uplink with self-recovery logic and a daily reboot at 01:00.
- **mavproxy.service** – Systemd unit file for running MAVProxy as a managed service.

## Diagnostic Tools & Utilities
# RealSense
- **tcap.py** - capture single frame from T265
- **dcap.py** - capture single frame from D4XX-series Depth Cameras
# LTE
- **uplink-diagnostic


## Legacy Repository Contents

- **ONICS2.py** – Re-rework of Thien94's `rs_to_mavlink.py` script with multicore support.
- **ONICS.py** – Complete rework of Thien94's script that adds logging and thread-management options.
- **uplink.sh** – Bash script to manage an LTE uplink with self-recovery logic and a daily reboot at 01:00.
- **logfix.py** – Post-processes `uplink.sh` logs into condensed, one-line-per-month reports.
- **install/** – Install helpers for dependencies, services, Intel RealSense, and ROS (including `install-deps.sh`, `install-services.sh`, `install_realsense.sh`, and `install_ros.sh`). _Currently untested._
- **hall-monitor.sh** – Watches a single service and restarts it if it crashes.
- **t265_precland_apriltag.py** – Optimized fork of Thien94's script of the same name. _Currently untested._
- **d4xx_to_mavlink.py** – Optimized fork of Thien94's script of the same name. _Currently untested._
- **d435i_to_mavlink.py** – Optimized fork of Thien94's script with added support for the D435i IMU. _Currently untested._

## Additional Notes

- Your Milage May Vary.
- Untested scripts may require tweaks or additional dependencies—use caution in production environments.
- When modifying systemd units or shell scripts, keep backups of working configurations so you can quickly revert.
