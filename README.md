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

The software is designed to be simulated using the [X-Plane 12][xplane-link] flight simulator. However, it was produced using real-world protocols, so  it can also be used with UAVCAN and MAVLINK compatible hardware (minor adjustments required).

## Requirements
* Windows 11.0+ or MacOS 13.0+
* Python 3.11+
    * `pycyphal` library
    * `pymavlink` library
* X-Plane 12.06+ (if using simulation)

> Not tested with older versions or Linux, but may work

## Installation

Clone this repo into a desired directory

```bash
git clone https://github.com/fmachanda/fmuas-main.git <desired-directory>
```

Initialize the [public_regulated_data_types][prdt-link] submodule

```bash
git submodule update --init --recursive --remote
```

X-Plane simulation requires the files from the [fmuas-xp][fmuas-xp-link] repo to be loaded into `/X-Plane 12/Aircraft`.

```bash
git clone https://github.com/fmachanda/fmuas-xp.git <xplane-aircraft-directory>
```

## Important Files

```bash
├── fmuas-main 
    ├── ...
    ├── uav
        ├── CONFIG.ini # You can change this!
        ├── uav_main.py # You can run this!
        └── xp_interface.py # You can run this!
    ├── gcs
        ├── CONFIG.ini # You can change this!
        ├── gcs_basic.py # Import this to a python terminal for CLI
        └── gcs_gui.py # You can run this!
    └── common
        ├──  data_types
            ├── public_regulated_data_types # MUST INIT THIS SUBMODULE!
            └── ...
        ├── MASTER_CONFIG.ini # DO NOT CHANGE THIS!
        ├── key.py # Shared mavlink encryption key
        └── ...
└── fmuas-xp
```


`uav_main.py` runs the UAV  
`xp_interface.py` runs the connection with X-Plane 12  
`gcs_gui.py` runs a GCS window  
`gcs_basic.py` can be used as a [CLI](#gcs-command-line-interface) if imported in a python terminal  
`CONFIG.ini` in `uav` and `gcs` contain customizable settings for UAV and GCS instances
`MASTER_CONFIG.ini` contains development settings that should not be changed by most users
`key.py` contains the shared custom key used by MAVLINK connections

> The `KEY = ...` line in `key.py` can be modified to any desired key (length 25). If using separate folders for GCS and UAV instances, ensure that this key is the same for both.

**Important:** `data_types/public_regulated_data_types` is a git [submodule][prdt-link] that must be initialized with:

```bash
git submodule update --init --recursive --remote
```

## Usage with X-Plane 12

To use with X-Plane 12:

1. Ensure that a clone of [fmuas-xp][fmuas-xp-link] is in `/X-Plane 12/Aircraft`

2. Launch X-Plane 12 ([free demo][xplane-link] will work)

3. Start a new flight with the FMUAS aircraft

    > The flight will remain paused until `xp_interface.py` starts running

4. Run the following scripts:
    * `gcs_gui.py`
    * `uas_controller.py`
    * `xp_interface.py`

5. Use the [GCS Interface](#gcs-command-line-interface) to boot and control the UAV.  

## GCS Command Line Interface

The `gcs_basic.py` script provides a very simple interface that can be used to connect with a UAV running `uav_main.py`. To use the provided commands, open a python terminal and type:

```python
import gcs_basic as gcs
```

To connect to a UAV, you need to know the System ID of the UAV's MAVLINK node. This can be found in the UAV's terminal. Connect to the UAV with:

```python
uav1 = gcs.Connect(<systemid>) # Replace <systemid> with the UAV's System ID
```

If you don't get a "Connected to ..." message, ensure that the CONFIG.ini files in the `uav` and `gcs` files are setting up compatible mavlink nodes.

To boot the UAV, run:

```python
uav1.boot()
```

To close the connection with the UAV, run:

```python
uav1.close()
```

Refer to the `gcs_basic.py` script for further documentation.

##

*Previous commit history (commits before 10/12/2023) can be found at [fmuas-main-history](https://github.com/fmachanda/fmuas-main-history)*

*Updated 10/19/2023*


[prdt-link]: https://github.com/OpenCyphal/public_regulated_data_types
[xplane-link]: https://www.x-plane.com/desktop/try-it/
[fmuas-xp-link]: https://github.com/fmachanda/fmuas-xp