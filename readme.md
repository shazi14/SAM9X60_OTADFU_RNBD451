---
parent: Harmony 3 application examples for SAM 9X60 family
title: OTA DFU using RNBD451 
has_children: false
has_toc: false
---

[![MCHP](https://www.microchip.com/ResourcePackages/Microchip/assets/dist/images/logo.png)](https://www.microchip.com)

# OTA DFU using RNBD451

This application shows how an image (harmony.bin) is transfrred from MBD App to MicroSD card of SAM9X60 Curiosity Board and upon reseting the board, SAM9x60 is booted into new imaage.

## Description

### Add Header to the Image:

- [Use HeaderAdd2Bin](firmware/Tool) c# utility to add header to harmony.bin. The header is a must for MBD app to recognize the image file. The output is harmony_out.bin

### Install MBD App on iOS and Android Phone

- Microchip Bluetooth Data (MBD) is available on Apple and google play and can be installed on the iOS and Android devices respectively.
- Send hamrmony_out.bin file to a web mail like gmail.com and yahoo.com 
- download harmony_out.bin file from the web mail and open it using MBD
- harmony_out.bin file will become visible in MBD App.

### Demo Setup

#### H/W Requirement

- SAM9x60 Curiosity Board
- RNBD451 Add On Board
- 5-inch Display Board (800 X 480)
- USB cable
- 1 GB or more microSD card
- Android/iOS devices with MBD App installed

#### S/W Requirement

- hamony_out.bin (new Image)
- sam_9x60_Rnbd451_sdcrad.X
- boot.bin

#### Build Image

- Build sam_9x60_Rnbd451_sdcrad.X



## Running the Demo 

- Copy harmony.bin and boot.bin file in microSD card.
- Insert microSD card in SAM9x60 curiosity board.
- Connect LCD Display with SAM9X60 curiosity board.
- Connect USB to power up SAM9X60 curiosity board.
- Press reset, yellow and red light will glow on RNBD451 Add On Board.
- Open MBD App and click on OTA DFU.
- Follow [instruction](https://onlinedocs.microchip.com/oxy/GUID-26457D23-798C-47B0-9F40-C5DA6E995C6F-en-US-2/GUID-EB08EAED-607F-4F3C-8C19-44608C8F8D25.html) to pair with the phone and start downloading the image.
- Green and Red LED will toggle on SAM9X60 Curiosity Board.
- Once Download Image is complete
- Reset the board
- SAM9X60 will boot into new Image and Legato Image will display on Display

Refer to the following table for LED name:

| Board | LED Name |
| ----- | -------- |
|  [SAM9X60-Curiosity board](https://www.microchip.com/en-us/development-tool/EV40E67A)  | RGB_LED(Green) |
|||
