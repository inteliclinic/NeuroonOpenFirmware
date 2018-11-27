# NeuroonOpenFirmware

## Getting started

**Necessary tools**:
- make
- arm-none-eabi-gcc
- arm-none-eabi-newlib
- nRF5x-Command-Line-Tools
- nrfutil

### Arch/Manjaro linux
```sudo pacman -S make arm-none-eabi-gcc arm-none-eabi-newlib yaourt python-pip```

```sudo yaourt -S nrf5x-command-line-tools``` or downlonad and install it form [here](https://www.nordicsemi.com/eng/nordic/download_resource/51386/31/93193012/94917)

```sudo pip install nrfutil```

### Ubuntu Linux
*TODO*
### MacOS
*TODO*
### Windows
*TODO*

*Windows is able to pass USB debugger to virtual machine(Virtual Box) and Neuroon can be flashed from within the vm(preferably Arch/Manjaro)*

## Flashing a device(on *\*nix* system) - software side
### Downloading code
```git clone https://github.com/inteliclinic/NeuroonOpenFirmware.git```

```cd NeuroonOpenFirmware```

```git submodule update --init```
### Compiling software
```make -j4```
### Flashing software on device (*no bootloader*)
```make flash flash_softdevice```

**Flashing bootloader**
*TODO*


## Flashing a device(on *\*nix* system) - hardware side
### Segger J-Link
This is the easiest way to flash Neuroon. J-Link debugger should be immediately recognized by your operating system.
It can be stand alone debuger or dev-board with onboard J-Link (like Nordic Semiconductor [PCA10040](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.nrf52%2Fdita%2Fnrf52%2Fdevelopment%2Fnrf52_dev_kit.html))

*TODO*:photos
### ST-Link
*TODO*
