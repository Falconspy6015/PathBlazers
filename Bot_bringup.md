
## Collaboration Protocols:

- Do not modify / delete /add any document/script to the raspi memory without affirmation.
## Safety Protocols:

- Always ensure the bot is in a safe position before turning it on. 
- Ensure the emergency stop button `the red one below the green switch` is used to stop the bot at times of emergencies
- Ensure no ROS node or joystick or keyboard of any device is actively sending cmd_vel data to the bot before turning it on.

## Turning on the bot:

Ensure the raspi is connected to the usb hub and the usb hub is connected to both the motor controllers before turning on the bot.

Use the green switch to turn on the bot.

### Debugging steps:

if the raspi doesnt glow a green continuous bright light :
- The battery voltage could have dropped significantly.
- The raspi has booting issues , ensure the sd card is inserted properly

## Step 1:

Connect all relevant devices to 
```
SSID: Abcd 
password : raspberry
```
If such a wifi does not exist , configure your mobile hotspot that way

## Step 2:
ensure your .bashrc file has the following export
```
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=10
```

## Step 3: 

To find Ip address to ssh into raspi do the following: 
```
sudo arp-scan --localnet | grep "d8:3a:dd:de:93:48"  
```
might have to do it a couple times aft bootup as raspi still would not have been discovered

ssh into the raspi

```
ssh pi@<ip_address>
```

usually you might not need the ip address and this might work
```
ssh pi@raspberrypi.local
```
## Step 4: Motor Control

 execute the following sequence of commands in the raspi to get the motors running

```
cd raspi

source install/setup.sh

ros2 launch navigator/navigator/run_all.py
```

## Step 5: Teleoperation control

#### Keyboard:

Run the following command on the device whose keyboard you want to teleop with

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

1) Run this on the device that has the Bluetooth controller attached for the Joystick

```
ros2 run navigator joy
```


2) Then you would have to run the following on a new terminal in raspi  ( follow steps 1 to 3) again.

and run the following in this new terminal of raspi 

```
cd raspi

source install/setup.sh

ros2 run navigator joy
```

## Step 6:  OFFLOADING Sensor data on a personal Laptop

### Lidar:

	Ensure you have sllidar package, to install it do the following

Pick a place where you want the local package to be installed and cd into it 

```
git clone https://github.com/Slamtec/sllidar_ros2.git
```

then source the overlay of this package everytime  u need to run lidar.

```
cd sllidar_ros2

source install/setup.sh

ros2 launch sllidar_ros2 sllidar_a1_launch.py
```

It should show up with the following message : current scan at XY hertz.

If it does not and throws up any error with code no 80008000 or 80018002 etc try unplugging the lidar wait for 10 seconds and try again.

some temporary fixes that might work:

```
sudo chmod +x /dev/ttyUSB*
```

### Depth_Cam:

To install the corresponding ros2 drivers for our oak-d s2 luxonis camera execute the following

```
sudo apt install ros-jazzy-depthai_ros
```


You would have to setup udev rule for the camera to ensure the package works. (GPT please)

Here is the command to launch camera

```
ros2 launch depthai_ros_driver camera.launch.py
```

Ensure if the topics are being published on a seperate terminal using

```
ros2 topic list
```

## Step 7: Running SLAM 

ensure you have slam_toolbox package , to install it do the following

```
sudo apt install ros2-jazzy-slam_toolbox
```

To run slam execute the following

```
ros2 launch slam_toolbox online_async_launch.py
```

If all tf transforms are aligned you should be able to visualise your map in rviz2 and the logs in the terminal output [Custom Lidar Detected]


Debugging steps if lifecycle node hasnt activated yet:

Todo:

## Step 8: Running VSLAM

ensure you have the following package installed to run rtabmap

```
sudo apt install ros2-jazzy-rtabmap_ros
```

and also ensure depthcamera is running : Step 6

To launch rtabmap:

```
ros2 launch depthai_ros_driver rtabmap.launch.py
```

## Step 9: To run ekf node:

ensure you have robot_localisation package installed

```
sudo apt install ros2-jazzy-robot_localisation
```

configure your odom sources in the ekf.yaml file before running.

To edit the yaml file:

```
sudo gedit /opt/ros/jazzy/share/robot_localisation/params/ekf.yaml
```

To run ekf node:

```
ros2 launch robot_localisation ekf.launch.py
```




#### Swapping imu parameters (optional):
- The imu topic is generated from the camera node of depth cam and the acceleration_x might be Gravitational acceleration 9.8ish while slam might be expecting acceleration_z to be that.

Here is a small script that you can save and run on your personal computer to swap the imu topic and publish /imu_swapped

```
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuAxisSwap(Node):
    def __init__(self):
        super().__init__('imu_axis_swap')
        self.sub = self.create_subscription(Imu, '/imu', self.callback, 10)
        self.pub = self.create_publisher(Imu, 'imu_swapped', 10)

    def callback(self, msg):
        # Swap X <-> Z
        msg.linear_acceleration.x, msg.linear_acceleration.z = msg.linear_acceleration.z, msg.linear_acceleration.x
        msg.angular_velocity.x, msg.angular_velocity.z = msg.angular_velocity.z, msg.angular_velocity.x
        self.pub.publish(msg)

rclpy.init()
rclpy.spin(ImuAxisSwap())

```


