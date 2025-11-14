# Aeronomicon

A collection of scripts and utilities to streamline working with Intel RealSense sensors, MAVLink, and ancillary tooling for aerial robotics projects.

## Repository Contents

- **ONICS2.py** – Re-rework of Thien94's `rs_to_mavlink.py` script with multicore support.
- **ONICS.py** – Complete rework of Thien94's script that adds logging and thread-management options.
- **uplink.sh** – Bash script to manage an LTE uplink with self-recovery logic and a daily reboot at 01:00.
- **logfix.py** – Post-processes `uplink.sh` logs into condensed, one-line-per-month reports.
- **install_realsense.sh** – Accelerates installation of Intel RealSense libraries. _Currently untested._
- **install_ros.sh** – Bootstraps ROS2 and `realsense-ros`. _Currently untested._
- **hall-monitor.sh** – Watches a single service and restarts it if it crashes.
- **apriltags3.py** – Upstream apriltags dependency preserved for compatibility.
- **t265_precland_apriltag.py** – Optimized fork of Thien94's script of the same name. _Currently untested._
- **d4xx_to_mavlink.py** – Optimized fork of Thien94's script of the same name. _Currently untested._
- **d435i_to_mavlink.py** – Optimized fork of Thien94's script with added support for the D435i IMU. _Currently untested._
- **mavproxy.service** – Systemd unit file for running MAVProxy as a managed service.

## Installing the MAVProxy Service

1. Copy the provided unit file into your systemd directory:
   ```bash
   sudo cp mavproxy.service /etc/systemd/system/mavproxy.service
   ```
2. Adjust any paths or environment variables in the unit file to match your installation.
3. Reload the systemd daemon to register the new service:
   ```bash
   sudo systemctl daemon-reload
   ```
4. Enable the service so it starts automatically on boot:
   ```bash
   sudo systemctl enable mavproxy.service
   ```
5. Start the service (or restart after changes):
   ```bash
   sudo systemctl start mavproxy.service
   ```
6. Check the status to confirm MAVProxy is running:
   ```bash
   systemctl status mavproxy.service
   ```

## Additional Notes

- The repository scripts were originally written for Ubuntu systems. Review each script before running it on other distributions.
- Untested scripts may require tweaks or additional dependencies—use caution in production environments.
- When modifying systemd units or shell scripts, keep backups of working configurations so you can quickly revert.

## Contributing

Issues and pull requests are welcome! Please describe the platform you tested on and include logs or configuration snippets that help reproduce your setup.
