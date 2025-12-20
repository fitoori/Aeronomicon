# Wireless Autonomous Terrestrial Navigation Experiment (W.A.T.N.E.)

The Wireless Autonomous Terrestrial Navigation Experiment (W.A.T.N.E.) is an
ongoing effort to design an Unmanned Aerial Vehicle (UAV) that is capable of
independently and precisely navigating both urban and rural terrestrial
environments.


## Introduction and disclaimers

This device is a scientific instrument and prototype. It is not intended to
cause harm or injury of any kind. The WATNE Project is not affiliated with any
government or larger organization and does not intend to pose a threat to
security, national or otherwise.

Although this vehicle is capable of being manually controlled, under normal
operation it is designed to self-operate within tolerances.

## Designated Emergency Landing Site (DELS)

```
Field by Lawrence Park C.I.
43.72220° N, 79.41201° W; altitude ≈ 180 m AMSL
```

Be advised that MAVLink expects lat/lon in degrees multiplied by 1e7 (int), and
altitude in millimeters (int).

```
home_lat = 437222000     # 43.72220 * 1e7
home_lon = -794120100    # -79.41201 * 1e7
home_alt = 180000        # 180 m AMSL -> 180000 mm
```

## Composition

WATNE is composed of several parts. This list may not be complete or remain
accurate for long.

- DJI Flame Wheel F550 Frame Kit + Motors
- XRotor 40A ESCs
- SBEC
- 4S LiPo Power Cell
- FrSky Backup Radio (for manual control)
- Netgear/Sierra Wireless 340U USB-LTE Modem
- HolyBro SiK Radio

- Intel T265 Tracking Camera
- Raspberry Pi 4B - 2GB RAM
- Navio2 AutoPilot HAT

## ONICS

To use Intel's RealSense T265 Tracking Camera (discontinued) and achieve
GPS-less precision loiter capabilities, outdated scripts were sourced and
revised to create ONICS: the Optical Navigation and Interference Control
System. This module is also responsible for AprilTag-based self-landing.

ONICS has been configured to use the following parameters:

```
APRILTAG_FAMILY = "tagStandard41h12"
tag_landing_id = 113
tag_landing_size = 0.144  # meters
```
