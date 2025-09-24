
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


## Step 5: Joystick control


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


