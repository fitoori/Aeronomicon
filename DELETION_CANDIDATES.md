# Deletion candidates

The following scripts appear broken or redundant and should be removed (or quarantined) before the next release.

## Broken setup scripts
- **install_realsense.sh** – The build/install step is backgrounded (`make && sudo make install &`), so the script continues and reports success even if the RealSense SDK fails to build. It also hard-codes ARM-specific Python 3.7 paths, which will fail on other platforms or Python versions. These two issues make the installer unreliable on most systems.
- **install_ros.sh** – The script mixes ROS 2 Foxy assets with a “Humble installation complete” message and uses a Bullseye Ubuntu apt stanza. The mismatched ROS releases, repository target, and completion banner indicate the instructions are inconsistent and likely to mis-install or fail outright.

## Redundant runtime script
- **ONICS.py** – Comments at the top note that this single-threaded version has been rewritten as ONICS 2. The maintained, multiprocessing version (ONICS2.py) should be kept instead of this legacy copy.
