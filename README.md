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
**TOOLCHAIN**\
Ubuntu 18.04 has a bug in repository version of 'newlib' which prevents output file from linking. ```arm-none-eabi-gcc``` and ```arm-none-eabi-newlib``` should be installed externally for example:

Download toolchain:\
```wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2018q2/gcc-arm-none-eabi-7-2018-q2-update-linux.tar.bz2```

Unpack:\
```tar xfv gcc-arm-none-eabi-7-2018-q2-update-linux.tar.bz2```

Now add newly installed toolchain to /usr/bin :\
```sudo ln -s `pwd`/gcc-arm-none-eabi-7-2018-q2-update/bin/* /usr/bin/.```

or alternatively modify ```NeuroonOpenFirmware/sdk/components/toolchain/gcc/Makefile.posix``` file to suit your needs.

**NRFTOOLS**\
First download [JLink Software](https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.deb), than install it using ```sudo dpkg -i JLink_Linux_x86_64.deb```. It is adviced to reboot your machine after installation.

Now download [nRF-Command-Line-Tools](https://www.nordicsemi.com/eng/nordic/download_resource/58852/31/84507885/94917): ```wget -O nRF-Command-Line-Tools_9_8_1_cd /icd //inux-x86_64.tar https://www.nordicsemi.com/eng/nordic/download_resource/58852/31/84507885/94917```

extract:\
```mkdir nrftools && cd nrftools```\
```mv ../nRF-Command-Line-Tools_9_8_1_Linux-x86_64.tar .```\
```tar xfv nRF-Command-Line-Tools_9_8_1_Linux-x86_64.tar```

add to binaries:\
```sudo ln -s `pwd`/nrfjprog/nrfjprog /usr/bin/.```

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
### Flashing software on device
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
