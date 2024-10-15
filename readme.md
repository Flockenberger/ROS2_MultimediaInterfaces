# Multimedia Interfaces Project
This repository contains my MMI project for WS24.
It is set-up in such a way that it is fairly easy to use, extend and build upon.

## Setup

For this setup to work follow the [ROS2 Windows Installation Guide](https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html)
Note: For this project I skipped QT as I don't need any fancy GUI etc.

This project assumes that you installed ROS2 (jazzy) in `C:\dev\ros2_jazzy` as per installation instructions!

## Code & Build
In `ros2_ws/` you will find 3 .bat files which I created for ease of use:

- Build.bat
    This file compiles the whole workspace -> meaning it compiles and installs all packages it finds in `ros2_ws/src`
    Additionally it also stays open so one can use it directly to start a ros2 node (for example).

- OpenVS.bat
    Since I primarily use Visual Studio for c++ development I made it such that this bat file sets up the local ROS2 environment within 
    the Visual Studio terminal and then opens the `./src/` folder - allowing CMake to find the ROS2 packages and the linter to work.
    Note: Theoretically you can skip the Build.bat file and simply compile directly in visual studio - but you need to add your compiled
    .exe file to the local environment yourself!

- OpenROSTerminal.bat
    Since more often than not one has multiple nodes to spin up, this simply opens a new terminal with a ROS2 local environment. 
    Note: This also calls install/setup.bat as well (otherwise your built nodes would not be visible to the current terminal)
    
