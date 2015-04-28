'''
@created: 20150427
@author: William Rowell
@contact: william.rowell@gmail.com
'''

import sys
import os
import time
import shutil
import subprocess
import datetime

OUTPUT_FOLDER = ''
LAUNCH_DELAY = 10 # time in s to wait before launching rostopic capture
EXPERIMENT_DURATION = 360 # duration in s, add extra time to start axoscope
ROS_ROOT = '/opt/ros/indigo/bin'
ROSLAUNCH = os.path.join(ROS_ROOT, 'roslaunch')
ROSBAG = os.path.join(ROS_ROOT, 'rosbag')


def datestamp(time = True):
    '''
    Return a datestamp string in format yyyymmddTHHMMSS by default.
    If time = False, format is yyyymmdd.
    '''
    if time: fmt = '%Y%m%dT%H%M%S'
    else: fmt = '%Y%m%d'
    return datetime.strftime(datetime.now(), fmt)


def main():
    '''
    USAGE: launch_kinefly_experiment.py <driver_name>

    Creates a unique experiment folder with the driver name, launches Kinefly,
    captures flystate data from all three cameras.
    '''
    # create experiment folder from datestamp, driver name?
    DATESTAMP = datestamp()
    BASENAME = DATESTAMP + '_' + sys.argv[1]
    experiment_folder = os.path.join(OUTPUT_FOLDER, BASENAME)
    try:
        os.umask(0002)
        os.mkdir(experiment_folder, 0775)
        os.umask(0022)
    except OSError:
        sys.exit('Cannot create folder %s in %s.' % DATESTAMP, OUTPUT_FOLDER)

    # copy kinefly.yaml files to experiment folder
    shutil.copy('kinefly1_pin.yaml', experiment_folder)
    shutil.copy('kinefly2_pin.yaml', experiment_folder)
    shutil.copy('kinefly3_pin.yaml', experiment_folder)

    # cd into experiment folder
    os.chdir(experiment_folder)

    # launch kinefly and capture video to bagfile
    kinefly_args = ' '.join(['RIG="kineflyjf"',
                             ROSLAUNCH,
                             'Kinefly',
                             'record.launch'])
    kinefly = subprocess.Popen(kinefly_args, shell=True)

    # wait for kinefly to launch completely
    time.sleep(LAUNCH_DELAY)

    # launch processes to capture flystate
    topic_list = ['/stimulus/ai',
                  '/kinefly1_pin/flystate',
                  '/kinefly2_pin/flystate',
                  '/kinefly3_pin/flystate']
    rosbag_args = ' '.join([ROSBAG,
                            'record',
                            '-O',
                            BASENAME + '_flystate'] +
                           topic_list)
    rosbag = subprocess.Popen(rosbag_args, shell=True)

    # wait for the experiment to end
    time.sleep(EXPERIMENT_DURATION)

    # end all processes gracefully
    rosbag.terminate()
    kinefly.terminate()

if __name__ == '__main__':
    main()


