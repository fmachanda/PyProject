![status](https://img.shields.io/badge/status-Development-orange)

# fmuas-main

> **Quick link**: [fmuas-xp][fmuas-xp-link]

**Table of Contents**
* [Background](#background)
* [Requirements](#requirements)
* [Installation](#installation)
* [Important Files](#important-files)
* [Usage with X-Plane 12](#usage-with-x-plane-12)
* [GCS Command Line Interface](#gcs-command-line-interface)

## Background
This project provides the software for a humanitarian remote-sensing UAV system.  

The software is designed to be simulated using the [X-Plane 12][xplane-link] flight simulator, but it also can be used with UAVCAN and MAVLINK compatible hardware (minor adjustments required).

## Requirements
* Windows 11.0+ or MacOS 13.0+
* Python 3.11+
    * `pycyphal` library
    * `pymavlink` library
* X-Plane 12.06+ (if using simulation)

> Not tested with older versions or Linux, but may work

## Installation

Download git repo as .zip

X-Plane simulation requires the files from the [fmuas-xp][fmuas-xp-link] repo.

## Important Files

    ├── fmuas-main 
        ├── ...
        ├── uas_controller.py << RUN THIS FOR UAV
        ├── gcs_gui.py << RUN THIS FOR GCS
        ├── xp_interface.py << RUN THIS TO USE WITH X-PLANE 12
        └──  data_types
            ├── public_regulated_data_types << MUST INIT THIS SUBMODULE
            └── custom_data_types
    └── fmuas-xp

## Usage with X-Plane 12

To use with X-Plane 12:

1. Ensure that a copy of [fmuas-xp][fmuas-xp-link] is in `/X-Plane 12/Aircraft`

2. Launch X-Plane 12

3. Start a new flight with the FMUAS aircraft

    > The flight will remain paused until `xp_interface.py` starts running

4. Run the following scripts:
    * `gcs_gui.py`
    * `uas_controller.py`
    * `xp_interface.py`

5. Use the [GCS Interface](#gcs-command-line-interface) to boot and control the UAV.  

## GCS Command Line Interface

No CLI functionality yet.

---

*Updated 10/19/2023*

[xplane-link]: https://www.x-plane.com/
[fmuas-xp-link]: https://github.com/fmachanda/fmuas-xp