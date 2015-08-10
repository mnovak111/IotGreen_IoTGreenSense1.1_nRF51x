# IotGreen_IoTGreenSense1.1_nRF51x

In this repository, you will find the source codes for the IotGreenProject, which can be found on Hackaday:
https://hackaday.io/project/6243-iotgreen-no-battery-iot-sensors-of-the-future

The software is written in Keil MDK ARM IDE. Free to use license which is limited to 32kB of code (the examples use maximum of 10kB so no need to worry) an be downloaded from the Keil website. Add the twi_hw_master_SD.c in Other folder to Source/twi_master/ folder of the nRF51 SDK.

I wrote the project on top of the ble_app_beacon example provided by Nordic Semiconductor. The SDK can be downloaded on Nordic website. You will have to link the libraries again and change the C/C++ tab of "Options for target" window. If you do not want to modify the software and just use the hex file, you will find it in /hex/ folder.

The licensing is described in LICENSE file.

