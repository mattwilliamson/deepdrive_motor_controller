# DeepDrive Motor Controller Firmware

## TODO

- [ ] restart esp32 if it can't connect
- [ ] Beeps/music
- [ ] Use multicore FreeRTOS to run motor loop on one core and micro ros loop on another
- [ ] Timeout on cmd_vel - stop the motors

## Hardware

- [Jetson Orin Nano](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)
- [Makerbase ESP32 Dual Brushless Micro FOC V1.0](https://makerbase3d.com/product/esp32-foc/)
- Hoverboard motors - got mine from Marketplace or you can do [something like this](https://a.aliexpress.com/_m0xGSQg)
- [Buck Converter](https://a.aliexpress.com/_m0FF8oU)
- [MOSFET](https://a.aliexpress.com/_mqzjWYk)
- [Power Button](https://a.aliexpress.com/_m0NGJUo)
- [Toggle Switch](https://a.aliexpress.com/_mMP37Ac)
- Battery - maybe [3S 11.1v or 4s 14.8v LiPo battery](https://a.aliexpress.com/_mqAFuBK)
- [XT60 connectors](https://a.aliexpress.com/_mr8JWhq)

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

### Set a symlink alias

To distinguish between other devices, like GPS

```sh
cat << EOF | sudo tee /etc/udev/rules.d/99-motor_controller.rules
KERNEL=="ttyUSB[0-9]*", SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", SYMLINK+="ttyMotor%n"
EOF

# Reload the rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Check it
```sh
matt@deepdrive:~$ ls /dev/ttyMotor*
/dev/ttyMotor1  /dev/ttyMotor2
```

Flash it!
(or just use VSCode PlatformIO serial monitor)
```sh
$ screen /dev/ttyMotor1 115200

Please enter 'left' or 'right' to set the side:
left

```

### Connect to ROS2

```sh
# source /opt/ros/${ROS_DISTRO}/install/setup.sh && \
# colcon build && \
# source $UROS_WS/install/setup.sh && \
# ros2 run micro_ros_setup create_agent_ws.sh && \
# ros2 run micro_ros_setup build_agent.sh

LEFT="/dev/ttyMotor1"
RIGHT="/dev/ttyMotor2"

ros2 run micro_ros_agent micro_ros_agent multiserial --devs "$LEFT $RIGHT" -v6
# ros2 run micro_ros_agent micro_ros_agent serial --dev $LEFT -v6
```

### Connect to ROS2 (Docker)
```sh
DISTRO="humble"
LEFT="/dev/serial/by-path/platform-3610000.xhci-usb-0:2.1:1.0-port0"
RIGHT="/dev/serial/by-path/platform-3610000.xhci-usb-0:2.3:1.0-port0"

sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:$DISTRO multiserial --devs "$LEFT $RIGHT" -v6

# sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/serial/by-path/platform-3610000.xhci-usb-0:2.4:1.0-port0 -v6

```

## Usage

### Show/echo topics

```sh
$ ros2 node list
/motor/right/motor_right_node

$ ros2 topic list
/motor/left/back/angle
/motor/left/back/vel/cmd
/motor/left/front/angle
/motor/left/front/vel/cmd
/motor/right/back/angle
/motor/right/back/vel/cmd
/motor/right/front/angle
/motor/right/front/vel/cmd
/parameter_events
/rosout

$ ros2 topic echo /motor/right/front/angle
data: 18
---
data: 18
---
data: 18
---
```

### Set radians/s velocity

```sh
$ ros2 topic info /motor/right/front/vel/cmd
Type: std_msgs/msg/Float32
Publisher count: 0
Subscription count: 1

$ ros2 topic pub --once /motor/right/front/vel/cmd std_msgs/Float32 '{"data":1.0}'

```

## Wiring

When connected directly to a 3s LiPo battery, I burned out 3 ESP32 modules before adding the buck converter. The buck converter has current limiting, eh8ch has also helped the USB bus from browning out.

The power button enables power theough the MOSFET for the whole system. The toggle switch turns on the power for the buck converter. This allows killing the motors if they go crazy or carch fire like mine did before I added the buck converter.

I did have to solder on a resistor for the power button LED.

For my throw switch, I had to mod it because the buck converter switch logic is inverterted. It is poowered on when the signal is pulled low. I just took the covere off and reversed it. I could have also just soldered a transistor here, which might be preferrable.


## Based on:

https://github.com/micro-ROS/micro_ros_platformio?tab=readme-ov-file#how-to-add-to-your-project

https://github.com/micro-ROS/micro_ros_platformio/blob/main/examples/micro-ros_publisher/src/Main.cpp

https://github.com/micro-ROS/micro-ROS-demos/tree/jazzy/rclc/int32_publisher_subscriber
