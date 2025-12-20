The Wireless Autonomous Terrestrial Navigation Experiment (W.A.T.N.E.)
is an ongoing effort to design an Unmanned Aerial Vehicle (UAV) that is
capable of independently & precisely navigating both urban & rural
terrestrial environments.


**Introduction and Disclaimers**
This device is a scientific instrument and prototype, and is not
intended to cause harm or injury of any kind. The WATNE Project is not
affiliated with any government or larger organization and does not
intend to pose a threat to security, national and otherwise.

Although this vehicle is capable of being manually controlled, under
normal operation it is designed to self-operate within tolerances.

**Designated Emergency Landing Site (DELS)**
###########################################################
#              Field by Lawrence Park C.I.                #
#    43.72220° N, 79.41201° W; altitude ≈ 180 m AMSL      #
###########################################################

Be advised that MAVLink expects lat/lon in degrees.

 * 1e7 (int), alt in mm (int)

home_lat = 437222000     # 43.72220 * 1e7
home_lon = -794120100    # -79.41201 * 1e7
home_alt = 180000        # 180 m AMSL -> 180000 mm

**Composition**
WATNE is composed of several parts, and this list may not be complete
or remain accurate for long:

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

**ONICS**
In order to use Intel's RealSense T265 Tracking Camera (discontinued)
to achieve GPS-less precision loiter capabilities, outdated scripts
were sourced and revised to create ONICS - The Optical Navigation and
Interference Control System. This module is also responsible for
AprilTag-based self-landing.

ONICS has been configured to use the following parameters:

APRILTAG_FAMILY = 'tagStandard41h12'
tag_landing_id = 113
tag_landing_size = 0.144 (Metres)
