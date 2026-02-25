These plugins allow to design UI for the nRF5340DK board + Adafruit 2.8'' TFT Touch V2 shield in Squareline Studio.
They create a Zephyr compatible project. v1.0.0 is for LVGL version 8.* and v2.0.0 is for LVGL version 9.*.

1. Create the Nordic directory inside the boards dir of Squareline Studio installation dir.
2. Copy the files into this new dir.

A device driver for the resistive touch TSC2007 based has been created. If you want to use it, you have to copy the contain of the zephyr directory to your Nordic NCS installation tree (e.g. /home/user/ncs/v3.2.1/zephyr). The zephyr project test_input_tsc2007 can be used to check that the driver works as expected.
