# Tail Calibration (`/calib`)

This directory contains scripts, tools, and configuration files used for calibrating the Tail UWB devices (anchors and tags). Calibration is crucial for accurate ranging and positioning.

## Key Files

*   **`calibrate.py`**: Likely the main Python script for performing calibration procedures, possibly involving range measurements, antenna delay calculation, or power calibration.
*   **`autocalib.sh`**: A shell script that probably automates parts of the calibration process, potentially calling other scripts like `calibrate.py`.
*   **`xtaltrim.py`**: A Python script specifically for calibrating the crystal oscillator trim value of the DW1000 radio. Accurate clock trimming is vital for TDOA systems.
*   **`ranging.py`**: A Python script likely used for performing ranging measurements between devices, possibly as part of the calibration process or for testing.
*   **`anchord.py`**: Python script for running an anchor daemon, possibly used during calibration to receive data from tags or other anchors.
*   **`anchor.service`**: A systemd service file for running `anchord.py` as a background service.
*   **`dwattr.py`**: Python script likely used to get/set DW1000 attributes via sysfs, similar to the functions in `python/tail.py` but perhaps tailored for calibration.
*   **`blinks.py`, `dwarf.py`, `tail.py`**: Copies or versions of the core Python libraries, potentially modified for calibration-specific tasks.
*   **`calib.json`, `setup.json`**: JSON configuration files likely holding calibration parameters, device setups, or results.
*   **`Makefile`**: Used to build the executable tools (`flashat`, `modhat`, `readhat`).
*   **`flashat`, `modhat`, `readhat`**: Compiled tools, likely for interacting with hardware during calibration (e.g., flashing firmware, modifying EEPROM on a Raspberry Pi HAT).
*   **`eepmod.py`**: Python script for modifying EEPROM data, likely on a Raspberry Pi HAT used with anchors.

## Usage

Refer to the specific scripts (`*.py`, `*.sh`) or potentially higher-level documentation for instructions on how to perform calibration using these tools. The `Makefile` might provide build instructions for the executable tools.
