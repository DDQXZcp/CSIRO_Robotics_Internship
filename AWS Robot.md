# AWS Robot

I have a Jetson Nano in my hand, but I don’t think it is powerful enough to run the SLAM algorithm or machine learning algorithm. 

Therefore I am going to use AWS Robomaker to assist me.

### Robomaker Document

[What Is AWS RoboMaker? - AWS RoboMaker](https://docs.aws.amazon.com/robomaker/latest/dg/chapter-welcome.html)

### Robomaker QuickStart

To begin with, you can create a new 'Robot Application' and 'Simulation Application' from the RoboMaker console, and then use the provided sample applications to learn how things work.

Here are some steps to start developing with AWS RoboMaker:

1. **Create a Robot Application:** This is essentially your robot's software stack. It contains the ROS nodes that make your robot work. If you don't have a robot application, you can use one of the sample applications provided by AWS.
2. **Create a Simulation Application:** This is the software stack that runs on the simulation. This can include the robot's model (URDF or SDF files), the world it will be simulated in, and any ROS nodes necessary for the simulation. Again, if you don't have a simulation application, you can use one of the sample applications.
3. **Create and Run a Simulation Job:** Once you have your robot and simulation applications, you can create a simulation job. This is where you define the resources for your simulation and specify the applications to use.

In the simulation job, AWS RoboMaker will launch a Gazebo simulation using the parameters specified in your simulation application. You can monitor the simulation through the RoboMaker console, and interact with it using standard ROS tools.

So, even if you only have experience running ROS on EC2, you can still use AWS RoboMaker to develop and test robot applications. The sample applications provided by AWS are a great starting point to learn how to structure your own applications and take advantage of the features provided by AWS RoboMaker.

## Preparing a Ubuntu PC

I need a Ubuntu PC because I don’t have one in my hand. Some of the ROS2 development processes needs a Linux environment.

### Method 1: AWS EC2 Free-tier (Failed to work, at least a bit time-consuming)

[Running Ubuntu Desktop on an AWS EC2 instance | Ubuntu](https://ubuntu.com/tutorials/ubuntu-desktop-aws#2-setting-up-tightvnc-on-aws)

In short, the Ubuntu provided does not have GUI. Need to 1. install ubuntu-desktop 2. install tight-vnc 3. Connect the desktop using VNC. 

It takes time and the free-tier computer is trash. It fails to run even a GUI. (30GB at most, 1GB flash)

### Method 2: AWS EC2 (Success and Quick, but it cost money)

I use the AWS Marketplace Image of Ubuntu 22.04 LTS with Gnome GUI and NICE DCV

Then in Command Line interface, set password, ubuntu is the username.

`sudo passwd ubuntu`

Then install NICE DCV, connect to the instance’s public DNS address and login with the username and password above.

## Robomaker Setup (Seems not working, Robomaker does not support iron for the time being)

### Container

Install the AWS CLI: [https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html)

Install Docker: [https://docs.docker.com/get-docker/](https://docs.docker.com/get-docker/)

`sudo apt update`

`sudo apt remove docker docker-engine [docker.io](http://docker.io/) containerd runc`

`sudo apt install apt-transport-https ca-certificates curl gnupg lsb-release`

`curl -fsSL [https://download.docker.com/linux/ubuntu/gpg](https://download.docker.com/linux/ubuntu/gpg) | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg`

`echo \
"deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] [https://download.docker.com/linux/ubuntu](https://download.docker.com/linux/ubuntu) \
$(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null`

`sudo apt update`

`sudo apt install docker-ce docker-ce-cli [containerd.io](http://containerd.io/)`

`sudo systemctl status docker`

Install the VCS Import Tool (if required for your workflow):

`sudo pip3 install vcstool`

Also to install pip3

sudo apt-get update

sudo apt-get -y install python3-pip

pip3 --version

### Building application containers from a ROS workspace

If the docker needs permission (replace ${USER} with ubuntu)

`sudo usermod -aG docker ${USER}`

`su - ${USER}`
Build the robot application with the following command:

```markdown
DOCKER_BUILDKIT=1 docker build . \
--build-arg ROS_DISTRO=iron \
--build-arg LOCAL_WS_DIR=./robot_ws \
--build-arg APP_NAME=helloworld-robot-app \
-t robomaker-helloworld-robot-app
```

After the robot application has been built, you can build the simulation application as follows:

```markdown
DOCKER_BUILDKIT=1 docker build . \
--build-arg GAZEBO_VERSION=gazebo-9 \
--build-arg ROS_DISTRO=melodic \
--build-arg LOCAL_WS_DIR=./simulation_ws \
--build-arg APP_NAME=helloworld-sim-app \
-t robomaker-helloworld-sim-app
```

## Setup the Gazebo and simulate Turtlebot (Decide to change back to Ubuntu 20 though Foxy has reached EOL)

From here it tells us several steps to simulate

Using Turtlebot 3 (community-contributed)

1. Getting started
2. Simulating
3. Navigating in simulation
4. Learning SLAM in simulation

[Demos — ROS 2 Documentation: Iron  documentation](https://docs.ros.org/en/iron/Tutorials/Demos.html)

Then follow the steps (Change all the foxy to iron)

[Simulate the TurtleBot3 | Ubuntu](https://ubuntu.com/blog/simulate-the-turtlebot3)

For the turtlebot3.repos, change the ros2-devel to foxy-devel, as described below

[https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/issues/65](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver/issues/65)

```markdown
repositories:
turtlebot3/turtlebot3:
type: git
url: [https://github.com/ROBOTIS-GIT/turtlebot3.git](https://github.com/ROBOTIS-GIT/turtlebot3.git)
version: foxy-devel
turtlebot3/turtlebot3_msgs:
type: git
url: [https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git](https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git)
version: foxy-devel
turtlebot3/turtlebot3_simulations:
type: git
url: [https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git](https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git)
version: foxy-devel
utils/DynamixelSDK:
type: git
url: [https://github.com/ROBOTIS-GIT/DynamixelSDK.git](https://github.com/ROBOTIS-GIT/DynamixelSDK.git)
version: foxy-devel
utils/hls_lfcd_lds_driver:
type: git
url: [https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git)
version: foxy-devel
utils/ld08_driver:
type: git
url: [https://github.com/ROBOTIS-GIT/ld08_driver.git](https://github.com/ROBOTIS-GIT/ld08_driver.git)
version: ros2-devel
```

Finally SLAM with turtlebot in simulation

[SLAM with TurtleBot3](https://github.com/cyberbotics/webots_ros2/wiki/SLAM-with-TurtleBot3)

Gazebo does not support iron?

## Successfully Simulating Lidar SLAM in Gazebo

[https://www.youtube.com/watch?v=8w3xhG1GPdo](https://www.youtube.com/watch?v=8w3xhG1GPdo)

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch turtlebot3_cartographer [cartographer.launch.py](http://cartographer.launch.py/) use_sim_time:=true

ros2 run turtlebot3_teleop teleop_keyboard

also, t3.medium is not enough, I change to c5.xlarge for more computation resources.

![Untitled](AWS%20Robot/Untitled.png)

[Gazebo  : Tutorial : ROS Depth Camera Integration](http://classic.gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros)
