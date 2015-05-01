# [kinefly_tools](https://github.com/williamrowell/kinefly_tools)
This is a collection of code related to the JRC DI implementation of Kinefly for screening.  Kinefly can be found [here](https://github.com/ssafarik/Kinefly), and depends on ROS, which can be found [here](http://www.ros.org/).

## [launch_kinefly_experiment.py](https://github.com/williamrowell/kinefly_tools/blob/master/launch_kinefly_experiment.py)
### USAGE: launch_kinefly_experiment.py [<driver_name>]

* creates unique experiment folder with datestamp and driver_name
* backs up yaml files containing ROI configuration to experiment folder
* saves a 6 minute rosbag with the following topics:
  * /stimulus/ai
  * /kinefly1_pin/flystate
  * /kinefly2_pin/flystate
  * /kinefly3_pin/flystate
  * /camera1/image_raw/compressed
  * /camera2/image_raw/compressed
  * /camera3/image_raw/compressed

### TODO:

* back up yaml files containing phidget configuration to experiment folder
* back up yaml files containing kinefly configuration to experiment folder
