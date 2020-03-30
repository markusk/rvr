# rvr
A ROS Python package for the Sphero RVR.

[![GitHub issues](https://img.shields.io/github/issues/markusk/rvr)](https://github.com/markusk/rvr/issues)
[![GitHub stars](https://img.shields.io/github/stars/markusk/rvr)](https://github.com/markusk/rvr/stargazers)
[![GitHub license](https://img.shields.io/github/license/markusk/rvr)](https://github.com/markusk/rvr/blob/master/LICENSE)
---
## 0. Important notice

_**This code is still in the middle of the development process!!**_

---
## 1. OS Setup
- Install [Ubuntu Mate](https://ubuntu-mate.org/download/) on your Raspberry Pi
- Install [ROS](http://wiki.ros.org/melodic/Installation/Ubuntu/) on your Ubuntu Mate

- [Enable SSH](https://askubuntu.com/questions/626372/could-not-load-host-key-etc-ssh-ssh-host-ed25519-key-in-var-log-auth-log/649782):
```
sudo ssh-keygen -A
```

- start ssh
```
sudo systemctl restart ssh.service
```

## 2. Joystick/Gamepad O support
```
sudo apt-get install joystick
```
### Microsoft XBOX Wireless Controller
Supported by the Kernel - if used with the USB connector.



## 3. ROS Setup
- install ROS
```
sudo apt-get install ros-melodic-ros-base
```

- install ROS packages
```
sudo apt-get install ros-melodic-urg-node ros-melodic-teleop-twist-keyboard joystick ros-melodic-joystick-drivers ros-melodic-teleop-twist-joy
```

- create a catkin workspace without 'src' folder:
```
mkdir catkin_ws
cd ~/catkin_ws
```

- create symbolic link with the name 'src', pointing to the 'src' folder in the ROS directory from this repository:
```
ln -s /home/$USERNAME/rvr/ROS/catkin_workspace/src/ src
catkin_make
```

---

## 3. Run ROS
### The main launch file
On the robot (Raspberry Pi):
```
roslaunch rvr rvr.launch
```
On another computer (the ground control center):
```
export ROS_MASTER_URI=http://<hostname>:11311
rosparam set joy_node/dev "/dev/input/js1"
```

_**to do:**_
```
roslaunch rvr ground_control_center.launch
```
---

## _**to do:**_ 4. Setting up ROS for autostart
### systemd under Ubuntu
```
sudo cp raspi/etc__systemd__system__rvr-ros-start.service /etc/systemd/system/rvr-ros-start.service
sudo systemctl daemon-reload
sudo systemctl start rvr-ros-start.service
sudo systemctl enable rvr-ros-start.service
```

---

## The ROS launch files

### K
#### keyboard_control_test
Listens to a teleop_twist_keyboard node and prints out the data/messages. Uses:
- _teleop_twist_keyboard_
- _nodes/keyboard_listener.py_

### M
#### motor_server
Controls the motors on the robot. Uses:
- _motor_server.py_

### R
#### rvr
Controls the whole robot. To be started on the robot. Uses:
_**to do:**_ _tf_broadcaster.py_
_**to do:**_  _battery_publisher.py_
_**to do:**_  _imu_bno055.py_
- _motor_server.py_
_**to do:**_  _base_controller.py_
_**to do:**_  _minibot_camera_
_**to do:**_  _urg_node_
