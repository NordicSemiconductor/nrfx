# nrfx

## Overview

nrfx is a standalone set of drivers for peripherals present in Nordic
Semiconductor's SoCs and SiPs. It originated as an extract from the nRF5 SDK.
The intention was to provide drivers that can be used in various environments
without the necessity to integrate other parts of the SDK into them.
For the user's convenience, the drivers come with the MDK package. This package
contains definitions of register structures and bitfields for all supported
SoCs, as well as startup and initialization files for them.

## Supported SoCs and SiPs

* nRF52805
* nRF52810
* nRF52811
* nRF52820
* nRF52832
* nRF52833
* nRF52840
* nRF5340
* nRF54L05
* nRF54L10
* nRF54L15
* nRF54LM20A
* nRF54LM20B
* nRF54LS05A
* nRF54LS05B
* nRF54LV10A
* nRF9131
* nRF9160
* nRF9161

## Directories

```
 .
 ├── bsp                   # Board Support Package
 │   └── stable            # BSP for stable platforms
 │       ├── mdk           # nRF MDK files
 │       ├── soc           # SoC specific files
 │       └── templates     # SoC specific templates of nrfx integration files
 ├── doc                   # Project documentation files
 ├── drivers               # nrfx driver files
 │   ├── include           # nrfx driver headers
 │   └── src               # nrfx driver sources
 ├── hal                   # Hardware Access Layer files
 ├── haly                  # Extended Hardware Access Layer files (deprecated)
 ├── helpers               # nrfx driver helper files
 ├── lib                   # nrfx internal libraries
 └── templates             # SoC-agnostic templates of nrfx integration files
```

## Generating documentation

nrfx documentation is available in the `doc\html` folder of the release package.

Refer to [this guide](doc/README.md) for more details.
