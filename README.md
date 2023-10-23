![status](https://img.shields.io/badge/status-Development-orange)

# fmuas-main

> Optional X-Plane files can be found at [fmuas-xp][fmuas-xp-link]

**Table of Contents**
* [Overview](#overview)
* [Requirements](#requirements)
* [Installation](#installation)
* [Important Files](#important-files)
* [Usage with X-Plane 12](#usage-with-x-plane-12)
* [GCS Command Line Interface](#gcs-command-line-interface)
* [Credits](#credits)

## Overview
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

X-Plane simulation requires the files from the [fmuas-xp][fmuas-xp-link] repo to be cloned into `X-Plane 12/Aircraft/`.

```bash
git clone https://github.com/fmachanda/fmuas-xp.git <aircraft-directory>
```

## Important Files

```bash
├── fmuas-main 
    ├── ...
    ├── uav
        ├── uav.py # Run this
        └── xpio.py # Run this
    ├── gcs
        ├── gcs.py # Use this in a python terminal
        └── gcs_gui.py # Run this
    └── common
        ├──  data_types
            ├── public_regulated_data_types # INIT THIS SUBMODULE!
            └── ...
        ├── CONFIG.ini # Change this
        ├── key.py # Shared mavlink encryption key
        └── ...
└── fmuas-xp
```


`uav/uav.py` runs the UAV  
`uav/xpio.py` runs the connection with X-Plane 12  

`gcs/gcs_gui.py` runs a GCS window  
`gcs/gcs.py` can be used as a [CLI](#gcs-command-line-interface) if imported in a python terminal  

`common/CONFIG.ini` contains changeable settings for UAV and GCS instances  

`common/key.py` contains the shared custom key used by MAVLINK connections
> The `KEY = ...` line in `key.py` can be changed to any desired MAVLINK key (must be length 25). If using separate folders for GCS and UAV instances, ensure that this key is the same for both.  

**Important:** `data_types/public_regulated_data_types/` is a git [submodule][prdt-link] that must be initialized with:

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
    * `uav.py`
    * `xpio.py`

5. Use the [GCS Interface](#gcs-command-line-interface) to boot and control the UAV.  

## GCS Command Line Interface

The `gcs.py` script provides a very simple interface that can be used to connect with a UAV running `uav.py`. To use the provided commands, open a python terminal and type:

```python
import gcs
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

Refer to the `gcs.py` script for further documentation.

---
### Credits

`common/data_types/public_regulated_data_types/` taken from [OpenCyphal/public_regulated_data_types][prdt-link]  

`common/data_types/custom_data_types/uavcan_archived/` taken from [dronecan/DSDL](https://github.com/dronecan/DSDL)  

`common/find_xp.py` copied from the [XPPython3 Docs](https://xppython3.readthedocs.io/en/latest/_static/find_xp.py)  

`GlobalRx.Att.quaternion_to_euler()` method in `uav/uav.py` copied from [automaticaddison](https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/)  

##

*Previous commit history (commits before 10/12/2023) can be found at [fmuas-main-history](https://github.com/fmachanda/fmuas-main-history)*

*Updated 10/20/2023*


[prdt-link]: https://github.com/OpenCyphal/public_regulated_data_types
[xplane-link]: https://www.x-plane.com/desktop/try-it/
[fmuas-xp-link]: https://github.com/fmachanda/fmuas-xp