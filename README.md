# Control ToolBox

In this repo, we build some common controller planner for robot arm involving kinematics and dynamics, and evaluating them in the symulation for my personal study. But, I do not provide ROS interface for now.

## Install
I provide docker for running the experiment here. The Simulation I used is CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04. You can download the other version and rename the code by your self.

Once download the .tar.xz file, place it into the docker/download directory. Then, you can build build the image by 
```
cd docker
./build.sh
```

## Launch the container
Once you have built the image, you can run following script to run the docker container
```
cd docker
./run.sh
```
Before that, you may need to run __xhost +__ to allow GUI for the terminal you want to use.

You can also use the following script to enter the container with another terminal
```
cd docker
./enter.sh
```

## Build the Project in Container
In the container, you will need to build the project for running the client to controll the robot in the Sim by the following code
```
cd /root/planner/coppeliaSimClient
mkdir build && cd build
cmake ..
make
```

##
In the following sections, I provide the tutorial to run different controllers for different purpose in the sim

1. [run timmer](tutorial/SIM_TIMER_README.md)