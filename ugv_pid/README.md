# Controlling an FMU of a UGV in ROS2 with PID Controllers
In this project, I run an FMU of a unmanned ground vehicle in ROS2. The UGV has a chassis model, tires, and an DC permanent magnet motor on either side. The motors are controlled by PID controllers. 
[image1](./doc/graph.png)
![alt_text][image1]

---
This [video](https://www.youtube.com/watch?v=vp9wAlj1cb8) shows the output of this workspace.

---
## Requirements
* [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Releases/Release-Foxy-Fitzroy.html)
* FMU binaries are for Linux
* [FMI Adaptor](https://github.com/boschresearch/fmi_adapter)
* [MapleSim Insight](https://www.maplesoft.com/products/maplesim-insight/) 2020.2 or higher for FMU visualization.

---
## Building the Project
* Clone this repo to your filesystem.
* Clone the [fmi_adaptor](https://github.com/boschresearch/fmi_adapter/tree/master/fmi_adapter) package to the `./src` folder.
* Navigate to the project folder and build the project using `colcon build`
* In a new terminal window, source the `./install/setup.sh` file and launch the main launch file using `ros2 launch ugv_launch ugv.launch.py`
* If you have MapleSim Insight installed, it will automatically pop up and will show the FMU visualization.
* In a new terminal window, source the `./install/setup.sh` file and run the `teleop` node using `ros2 run command teleop`
* Use the keyboard arrow keys to propel and steer the UGV.

