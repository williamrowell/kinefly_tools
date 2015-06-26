#!/usr/bin/python

"""
@created: 20150427
@edited: 20150501
@author: William Rowell
@contact: william.rowell@gmail.com
"""

import sys
import os
import time
import shutil
import subprocess
import signal
from datetime import datetime


CONFIG_FOLDER = '/home/kineflyjf'
OUTPUT_FOLDER = '/home/kineflyjf/screen_data'
EXPERIMENT_DURATION = 360  # duration in s, add extra time to start axoscope
ROS_ROOT = '/opt/ros/hydro/bin'
ROSBAG = os.path.join(ROS_ROOT, 'rosbag')
ROSPARAM = os.path.join(ROS_ROOT, 'rosparam')
TOPIC_LIST = ['/stimulus/ai',
              '/kinefly1_pin/flystate',
              '/kinefly2_pin/flystate',
              '/kinefly3_pin/flystate',
              '/camera1/image_raw/compressed',
              '/camera2/image_raw/compressed',
              '/camera3/image_raw/compressed']


def terminate_process_and_children(p):
    """
    terminate_process_and_children(Popen) -> None

    Find all children of Popen process and terminate process and children.
    """
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
    p.terminate()


def datestamp(time = True):
    """
    datestamp(time = True) -> str

    Return a datestamp string in format yyyymmddTHHMMSS by default.
    If time = False, format is yyyymmdd.
    """
    if time: fmt = '%Y%m%dT%H%M%S'
    else: fmt = '%Y%m%d'
    return datetime.strftime(datetime.now(), fmt)


def main():
    """
    USAGE: launch_kinefly_experiment.py [<driver_name>]

    Creates a unique experiment folder with the driver name,
    captures flystate data, video, and analog stimulus information.
    """
    # create experiment folder from datestamp, driver name?
    DATESTAMP = datestamp()
    if len(sys.argv) > 2:
        BASENAME = DATESTAMP + '_' + sys.argv[1]
    else:
        BASENAME = DATESTAMP + '_' + 'test'
    experiment_folder = os.path.join(OUTPUT_FOLDER, BASENAME)
    try:
        os.umask(0002)
        os.mkdir(experiment_folder, 0775)
        os.umask(0022)
    except OSError:
        sys.exit('Cannot create folder %s in %s.' % DATESTAMP, OUTPUT_FOLDER)

    # cd into experiment folder
    os.chdir(experiment_folder)

    # launch processes to capture flystate
    rosbag_args = [ROSBAG, 'record', '-O', BASENAME] + TOPIC_LIST
    rosbag = subprocess.Popen(rosbag_args)

    # wait for the experiment to end
    if len(sys.argv) > 2:
        time.sleep(int(sys.argv[-1]))
    else:
        time.sleep(EXPERIMENT_DURATION)

    # dump all ros parameters to rosparam.yaml
    rosparam_args = [ROSPARAM, 'dump', 'rosparam.yaml']
    rosparam = subprocess.Popen(rosparam_args, shell=False)

    # end all processes gracefully
    terminate_process_and_children(rosbag)

if __name__ == '__main__':
    main()


