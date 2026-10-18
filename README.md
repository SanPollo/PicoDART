# PerryDART - Amstrad CPS8256 Emulator

## About

PerryDART is an Amstrad CPS8256 emulator for the Raspberry Pi Pico microcontroller. It represents the firmware for the serial interface part of [Project PerryFi](#project-perryfi).

It is based on the [JOYCE](https://www.seasip.info/Unix/Joyce/) Amstrad PCW emulator by John Elliott.

PerryDART is licensed under the GNU General Public License Version 3 or any later version. Please see the [LICENSE](LICENSE) file for more information.

<br />

## Usage

1. Download the [latest release](https://github.com/SanPollo/PicoDART/releases/latest) of PerryDART in **.uf2** format.
2. Disconnect the PerryFi 2 from the PCW and any other power sources.
3. Acquire a USB-C to USB, or micro USB to USB **data cable** (depending on your Pico). Note that power-only cables designed for charging **will not work**.
4. Connect the relevant end to the Pico.
5. Hold the **BOOTSEL** button down on the Pico, and plug the cable into your PC, keeping the **BOOTSEL** button held down.
6. When the File Explorer window opens showing the contents of the Pico, release the **BOOTSEL** button.
7. Copy the **.uf2** file you downloaded in step 1 onto the Pico.
8. When the Pico's File Explorer window disappears, disconnect the cable from both the Pico and the PC.

<br />

## Project PerryFi

Project PerryFi consists of several separate components which make the PerryFi 2 possible:

* [PerryFi 2](https://github.com/SanPollo/PerryFi2) - Hardware
* [PerryDART](https://github.com/SanPollo/PicoDART) - Pico Firmware
* [PerryZi](https://github.com/SanPollo/PerryZi) - ESP8266 Firmware
