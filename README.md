# [kinefly_tools](https://github.com/williamrowell/kinefly_tools)
This is a collection of code related to the JRC DI implementation of Kinefly.  Kinefly can be found [here](https://github.com/ssafarik/Kinefly), and depends on ROS, which can be found [here](http://www.ros.org/).

## [launch_kinefly_experiment.py](https://github.com/williamrowell/kinefly_tools/blob/master/launch_kinefly_experiment.py)
### `USAGE: launch_kinefly_experiment.py [<exp_name>]`

* creates unique experiment folder with datestamp and exp_name
* backs up all ROS configuration state to experiment folder
* saves a 6 minute rosbag with the following topics:
  * /stimulus/ai
  * /kinefly1_pin/flystate
  * /kinefly2_pin/flystate
  * /kinefly3_pin/flystate
  * /camera1/image_raw/compressed
  * /camera2/image_raw/compressed
  * /camera3/image_raw/compressed

### TO DO:


## [unpack_bag.py](https://github.com/williamrowell/kinefly_tools/blob/master/unpack_bag.py)
### `USAGE: unpack_bag.py <bag_file>`

* opens bag_file created by launch_kinefly_experiment.py
* creates hdf5 file to store experimental data
* retrieves and stores stimulus information, images, and specific flystate information in hdf5 file.

### TO DO:

* get around lack of `.get_types_and_topics()` in previous implementations of rosbag.Bag
* decide which flystate information to retrieve
* handle multidimensional flystate information (like arrays of angles)
* implement lossless compression in hdf5 file
