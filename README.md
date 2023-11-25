[![status](https://img.shields.io/badge/status-Development-orange)](https://trello.com/b/E168SpHn/fmuas)

# fmuas-main

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
* Python 3.11.6+
    * **pip** modules listed in `requirements.txt`
* X-Plane 12.06+ (if using simulation)

> Not tested with older versions or Linux, but may work

## Installation

Clone this repo into the desired directory. This project contains the [public_regulated_data_types][prdt-link] submodule, so you must use `--recurse-submodules`.

The X-Plane simulation requires that the files are placed into the `X-Plane 12/Aircraft/` directory. Replace `<directory>` below with the correct path.

```bash
git clone https://github.com/fmachanda/fmuas-main.git <directory> --recurse-submodules

cd <directory>/fmuas-main
```

Install and update required python modules with [pip](https://pip.pypa.io/en/stable/installation/).

```bash
python -m pip install -r requirements.txt --upgrade
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
        └── gui.py # Run this
    ├── common
        ├── public_regulated_data_types # INIT THIS SUBMODULE!
        ├── config.ini # Change this
        ├── key.py # Shared mavlink encryption key
        └── ...
    └── fmuas-xp # Contains the X-Plane files for simulation
```


`uav/uav.py` runs the UAV  
`uav/xpio.py` runs the connection with X-Plane 12  

`gcs/gui.py` runs a GCS window  
`gcs/gcs.py` can be used as a [CLI](#gcs-command-line-interface) if imported in a python terminal

`fmuas-xp` contains the X-Plane aircraft files

`common/config.ini` contains changeable settings for UAV and GCS instances  

`common/key.py` contains the shared custom key used by MAVLINK connections
> The `KEY = ...` line in `key.py` can be changed to any desired MAVLINK key (must be length 25). If using separate folders for GCS and UAV instances, ensure that this key is the same for both.  

**Important:** `common/public_regulated_data_types/` is a git [submodule][prdt-link] that must be initialized for scripts to work

## Usage with X-Plane 12

To use with X-Plane 12:

1. Ensure that the `fmuas-main` folder is in `/X-Plane 12/Aircraft`

2. Launch X-Plane 12 ([free demo][xplane-link] will work)

3. Start a new flight with the FMUAS aircraft

    > The flight will remain paused until `xpio.py` starts running

4. Run the following scripts from the terminal:
    * `gcs/gui.py`
    * `uav/uav.py`
    * `uav/xpio.py`

5. Use the [GCS Interface](#gcs-command-line-interface) to boot and control the UAV.  

## GCS Command Line Interface

The `gcs.py` script provides a very simple interface that can be used to connect with a UAV running `uav.py`. To use the provided commands, open a python terminal and type: -->

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

`common/public_regulated_data_types/` cloned from [OpenCyphal/public_regulated_data_types][prdt-link]

`common/find_xp.py` copied from the [XPPython3 Docs](https://xppython3.readthedocs.io/en/latest/_static/find_xp.py)  

`quaternion_to_euler()` and `euler_to_quaternion()` functions in `common/angles.py` copied from [automaticaddison](https://automaticaddison.com)

`xlua.xpl` and `init.lua` in `fmuas-xp/plugins/` copied from X-Plane

***All  other code is original.***

##

*Previous commit history (commits before 10/12/2023) can be found at [fmuas-main-history](https://github.com/fmachanda/fmuas-main-history)*

*Previous X-Plane file repo (before 11/18/2023) can be found at [fmuas-xp][fmuas-xp-link]*

*Updated 11/18/2023*


[prdt-link]: https://github.com/OpenCyphal/public_regulated_data_types
[xplane-link]: https://www.x-plane.com/desktop/try-it/
[fmuas-xp-link]: https://github.com/fmachanda/fmuas-xp