# Tail RTLS (`/rtls`)

This directory contains Python scripts and subdirectories related to the Real-Time Locating System (RTLS) aspects of the Tail project. It includes tools for testing, configuration, data analysis, and potentially core RTLS algorithms.

## Key Files & Directories

*   **`tdoa.py`**: Likely contains implementations of Time Difference of Arrival (TDOA) algorithms for position calculation. This might be a copy or specific version of the main `python/tdoa.py`.
*   **`twrtest.py`**: Script for testing Two-Way Ranging (TWR) between devices.
*   **`owrtest.py`**: Script for testing One-Way Ranging (OWR) or blink-based TDOA.
*   **`config.py`**: Python script possibly related to loading or managing RTLS configurations (e.g., anchor positions, system parameters).
*   **`dump.py`**: Script for dumping raw data received from anchors/tags, likely for debugging or offline analysis.
*   **`allan-error.py`**: Script for calculating Allan deviation, a measure of frequency stability, likely used for analyzing clock drift in the UWB timestamps.
*   **`delay.py`**: Purpose unclear, might be related to measuring or compensating for processing delays.
*   **`xtalt.py`**: Possibly related to crystal trim calibration or testing its effect on RTLS.
*   **`blinks.py`, `tail.py`, `dwattr.py`**: Copies or specific versions of core Python libraries used within the RTLS context.
*   **`Makefile`**: Might be used for running tests or other RTLS-related tasks.
*   **`calibration/`**: Subdirectory potentially containing scripts or data specifically for RTLS calibration (e.g., anchor position survey).
*   **`deprecated/`**: Contains older or unused RTLS-related code.
*   **`plotting/`**: Scripts for visualizing RTLS data, positions, or performance metrics.
*   **`rfcomp/`**: Likely related to RF (Radio Frequency) compensation based on signal strength or other factors.
*   **`tools/`**: Miscellaneous utility scripts for RTLS tasks.
*   **`xtalt-comp/`**: Possibly related to compensating for crystal oscillator drift (xtalt) in RTLS calculations.

## Overview

This directory seems central to the positioning aspect of the Tail system, providing tools to test different ranging methods (TWR, TDOA/OWR), analyze system performance (Allan deviation, clock drift), and potentially run the core positioning engine itself (though the main server might reside elsewhere, e.g., `tail-server` repo).
