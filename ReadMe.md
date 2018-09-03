# Pointgrey Ladybug ROS Driver

This driver is an expansion of the Ladybug driver in [Autoware](https://github.com/CPFL/Autoware/tree/master/ros/src/sensing/drivers/camera/packages/pointgrey) driver.
The key changes are more config options and added support for the Ladybug 5+ camera.
See below for the install instructions, and the parameters that can be set in the launch file.
Also this prints out more information about the configuration, so you should check if your camera is running on USB2 or USB3 according to the SDK library.


## Installation
* Download SDK - https://www.ptgrey.com/Downloads/GetSecureDownloadItem/10997
* `sudo apt-get install xsdcxx`
* Extract SDK to get deb file
* `sudo dpkg -i ladybug-1.15.3.23_amd64.deb`
* Open the /etc/default/grub file in any text editor. Find and replace:
    * `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"`
    * With:
    * `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"`
* `sudo update-grub`
* restart computer
* check that it is enabled: `cat /sys/module/usbcore/parameters/usbfs_memory_mb`



## Parameters


* `framerate` - framrate of the camera (example 10-20 fps)
* `shutter_time` - time in second the shutter should be open (example 0.02-2 seconds)
* `gain` - amount of gain the image should have applied (example 0-18 db)


