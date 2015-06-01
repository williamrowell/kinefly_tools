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

## [Stimulus.m](https://github.com/williamrowell/kinefly_tools/blob/master/Stimulus.m)
### `USAGE: Stimulus( [ pulse_length pulse_period num_pulses trial_length num_trials voltage ] )`
```
%STIMULUS Sends a pulse over AO to control red LED.
%   pulse_length    width of one light pulse in s, float
%   pulse_period    width of one pulse period in s, float
%   num_pulses      number of pulses per trial, int
%   trial_length    total duration of one trial in s, int/float
%   num_trials      number of replicates, int
%   voltage         output voltage, must be in range 0 <= v <= ~4.2>, float
```

## [Interval.m](https://github.com/williamrowell/kinefly_tools/blob/master/Interval.m)
### `USAGE: Interval( [ pattern_id mode_array gain_bias_array stimulus_array pre_delay post_delay ] )`
```
%INTERVAL One trial or rest interval.
%   pattern_id         integer between 0 and 255 corresponding to visual pattern
%   mode_array         [x y] 0 = open loop, 1 = closed loop
%   gain_bias_array    [x_gain x_bias y_gain y_bias]
%   stimulus_array     [pulse_length pulse_period num_pulses trial_length num_trials voltage]
%   pre_delay          time in s to display pattern before stimulus begins
%   post_delay         time in s to display pattern after stimulus begins
```

## [kinefly_experiment.m](https://github.com/williamrowell/kinefly_tools/blob/master/kinefly_experiment.m)
### `USAGE: kinefly_experiment( protocol_name, protocol_type, num_replicates, driver )`
```
%KINEFLY_EXPERIMENT General use function to drive an experiment with a sequence of trial components
%   protocol_name      name of the .mat file containing the individual protocol components to use
%   protocol_type      if components are A, B, C, then collated (ABCABCABC), grouped (AAABBBCCC), or random
%   num_replicates     number of replicates of each components
%   driver             name of the driver line to be used when saving a record
```

## `*.mat`
Each contains a struct array of trial components for a protocol.  Each item in the struct has the following fields:
* pattern_id
* mode_array
* gain_bias_array
* stimulus_array
* pre_delay
* post_delay

The last indexed item is always the “rest” interval used between trials.
