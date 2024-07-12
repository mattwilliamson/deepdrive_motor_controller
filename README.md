# DeepDrive Motor Controller Firmware

## Hardware
- [Jetson Orin Nano](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)
- [Makerbase ESP32 Dual Brushless Micro FOC V1.0](https://makerbase3d.com/product/esp32-foc/)


## Software

- [Micro ROS](https://micro.ros.org/) - ROS2 Communication
- [SimpleFOC](https://docs.simplefoc.com/library_platformio) - Brushless motor controller library


## Setup
### Install PlatformIO

https://docs.platformio.org/en/latest/core/installation/index.html

```sh
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
```

### Clone
```sh
git clone git@github.com:mattwilliamson/deepdrive_motor_controller.git
cd deepdrive_motor_controller
echo 'PATH="$HOME/.platformio/penv/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
pio run
```

### Set EEPROM/Flash to store which motor it is
Once the program is flashed, we will need to connect one motor controller at a time and then flash the eeprom (emulated in flash) with the value of which side it is on.

Plug in MKS ESP32 FOC while powered up and get device details **one at a time**:

```sh
sudo dmesg -w

[  688.466785] usb 1-2.1: new full-speed USB device number 4 using tegra-xusb
[  688.573443] usb 1-2.1: New USB device found, idVendor=1a86, idProduct=7523, bcdDevice= 2.64
[  688.573452] usb 1-2.1: New USB device strings: Mfr=0, Product=2, SerialNumber=0
[  688.573457] usb 1-2.1: Product: USB Serial
[  688.611179] usbcore: registered new interface driver ch341
[  688.611200] usbserial: USB Serial support registered for ch341-uart
[  688.611241] ch341 1-2.1:1.0: ch341-uart converter detected
[  688.618337] usb 1-2.1: ch341-uart converter now attached to ttyUSB0

[ 1181.973673] usb 1-2.4: new full-speed USB device number 5 using tegra-xusb
[ 1182.080331] usb 1-2.4: New USB device found, idVendor=1a86, idProduct=7523, bcdDevice= 2.64
[ 1182.080340] usb 1-2.4: New USB device strings: Mfr=0, Product=2, SerialNumber=0
[ 1182.080344] usb 1-2.4: Product: USB Serial
[ 1182.082561] ch341 1-2.4:1.0: ch341-uart converter detected
[ 1182.089568] usb 1-2.4: ch341-uart converter now attached to ttyUSB1
```

Get device by port number
```sh
matt@deepdrive:~$ ls -l /dev/serial/by-path/
total 0
lrwxrwxrwx 1 root root 13 Jul 12 09:36 platform-3610000.xhci-usb-0:2.1:1.0-port0 -> ../../ttyUSB0
lrwxrwxrwx 1 root root 13 Jul 12 09:36 platform-3610000.xhci-usb-0:2.4:1.0-port0 -> ../../ttyUSB1
```

Flash it!
```sh
$ screen /dev/ttyUSB0

Please enter 'left' or 'right' to set the side:
left

```


### Connect to ROS2
```sh
DISTRO="humble"
LEFT="/dev/serial/by-path/platform-3610000.xhci-usb-0:2.1:1.0-port0"
RIGHT="/dev/serial/by-path/platform-3610000.xhci-usb-0:2.4:1.0-port0"
sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:$DISTRO multiserial --devs "$LEFT $RIGHT" -v6

# sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/serial/by-path/platform-3610000.xhci-usb-0:2.4:1.0-port0 -v6

```



## Based on:

https://github.com/micro-ROS/micro_ros_platformio?tab=readme-ov-file#how-to-add-to-your-project

https://github.com/micro-ROS/micro_ros_platformio/blob/main/examples/micro-ros_publisher/src/Main.cpp

https://github.com/micro-ROS/micro-ROS-demos/tree/jazzy/rclc/int32_publisher_subscriber
