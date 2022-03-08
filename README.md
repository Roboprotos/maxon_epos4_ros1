maxon_epos4_ros1
================

This package contains configuration files and sample code to use maxon EPOS4 Positioning Controllers with ROS1 using ros_canopen.

The license used is [![License: BSD 3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause) for all the packages.

The branch master has been tested with ROS Melodic.

The corresponding ROS Wiki page can be found [here](http://wiki.ros.org/maxon_epos4_ros1).

> :warning: **Important notes**: <p>This document and all provided sample code has been developed on behalf of maxon motor
ag. Any installation steps and code samples are intended for testing purposes only.
Please adapt the code to the needs of your concrete application.
Any warranty for proper functionality is excluded and has to be ensured based on tests within
the concrete system environment.<p>
**Take care of the wiring and safety instructions mentioned by the "Hardware Reference" of the
corresponding controller!**<p>
Please submit a request on maxon’s Support Center (-> http://support.maxongroup.com) in
case of any questions concerning the controller’s setup, wiring, or testing.
  
## Installation steps: 

- Open a terminal to download this package (master branch by default):
```
  $ git clone https://github.com/Roboprotos/maxon_epos4_ros1
```
Alternatively, you can clone a specific branch like melodic-devel (for ROS melodic), like this:
```
  $ git clone -b melodic-devel https://github.com/Roboprotos/maxon_epos4_ros1
``` 
- Look at the PDF file located in the documentation folder for information on how to use this package. It contains a complete tutorial on how to setup both the EPOS4 controllers and ROS1 along with ros_canopen and MoveIt! packages, with practical examples using the NVIDIA Jetson TX2 and Raspberry Pi embedded computers.

<br/>
  
![EPOS4_24_1 5_Compact-2_small](https://user-images.githubusercontent.com/66867384/143435983-b63cc9e0-8983-4f60-9cd4-cbe4f84f76f2.jpg)
  
![maxon_Logo_small](https://user-images.githubusercontent.com/66867384/143436010-5d3968e7-79f8-48b9-a35b-e5159ec20ee6.png)
