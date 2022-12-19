# Pointgrey Ladybug ROS Driver

This driver is an expansion of the Ladybug driver in [Autoware](https://github.com/autowarefoundation/autoware_ai_drivers/tree/master/pointgrey) driver.
The key changes are more config options and added support for the Ladybug 5+ camera.
See below for the install instructions, and the parameters that can be set in the launch file.
Also this prints out more information about the configuration, so you should check if your camera is running on USB2 or USB3 according to the SDK library.




## Launch Parameters


* `framerate` - framerate of the camera (example 10-20 fps)
* `shutter_time` - time in second the shutter should be open (example 0.02-2 seconds)
* `gain` - amount of gain the image should have applied (example 0-18 db)



## Installation with Docker


Create temp docker to play with:
```
DOCKER_CATKINWS=/home/patrick/workspace/catkin_ws_ladybug
docker run -t -i --privileged -v /dev/bus/usb:/dev/bus/usb \
    --mount type=bind,source=$DOCKER_CATKINWS,target=/catkin_ws \
    osrf/ros:kinetic-desktop-full bash
```


Docker file to actually create:
```
osrf/ros:kinetic-desktop-full

# system packages we need for the build
RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y \
    git wget autoconf automake \
    libopencv-dev \
    usbutils udev xsdcxx \
    python-catkin-tools


# Create the workspace and build kalibr in it
ENV WORKSPACE /catkin_ws

RUN mkdir -p $WORKSPACE/src && \
    cd $WORKSPACE && \
    catkin init && \
    catkin config --extend /opt/ros/kinetic && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release


# install our sdk
RUN dpkg -i $WORKSPACE/src/pointgrey_ladybug/debs/ladybug-1.16.3.48_amd64.deb

yes
root
yes


```



## Installation from Source (Ubuntu 16.04 only!)


* Download SDK from the Flir website
    * https://www.flir.com/products/ladybug-sdk/
    * https://flir.app.boxcn.net/v/LadybugSDK?pn=Ladybug+SDK&vn=Ladybug_SDK
* Required dependency `sudo apt-get install xsdcxx`
* Extract SDK to get deb file which we install `sudo dpkg -i ladybug-1.16.3.48_amd64.deb`
* Open the /etc/default/grub file in any text editor. Find and replace:
    * `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"`
    * With:
    * `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=1000"`
* `sudo update-grub`
* Restart computer (IMPORTANT!)
* Check that it is enabled: `cat /sys/module/usbcore/parameters/usbfs_memory_mb`
* Check that it shows up `lsusb` should have `1e10:3800 Point Grey Research, Inc. Ladybug5 LD5P-U3-51S5C-44` in it




## Uninstalling

* The package should be installed as a system package
* If you need to use other pointgrey drivers (that also use the libflycapture.so file)
* `sudo apt-get remove ladybug`







