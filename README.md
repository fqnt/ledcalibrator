# ledcalibrator
ROS module for automatic camera-based localization of LEDs on lightstrip


## Install (tested with ROS kinetic)

* Create a ROS catkin_ws following http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
* Load dependencies (e.g. by cloning them into the src dir)
  * pyledstrip for controling the lightstrip (https://github.com/cipold/pyledstrip) (currently, the python2 branch is needed!)
  * usb_cam for laptop integrated webcams (https://github.com/bosch-ros-pkg/usb_cam) 
* Checkout this repository into the src dir:
```bash
git clone https://github.com/fqnt/ledcalibrator.git
```
* Build it
```bash
catkin_make
```
* Run the demo (using live images from usb_cam)
```bash
source devel/setup.bash
roslaunch ledcalibrator demo.launch
```


## Known Issues

pyledstrip is not yet found automatically. Adjust the path in the main python file.
