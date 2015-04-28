'''
@date: 20150427
@author: William Rowell
@email: william.rowell@gmail.com
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


def datestamp(time = True):
    '''
    Return a datestamp string in format yyyymmddTHHMMSS by default.
    If time = False, format is yyyymmdd.
    '''
    if time: fmt = '%Y%m%dT%H%M%S'
    else: fmt = '%Y%m%d'
    return datetime.strftime(datetime.now(),fmt)


def rostopic_capture(topic, filename):
    '''
    rostopic_capture(topic, filename) -> Popen

    Open a shell process to subscribe to topic and store in filename.
    '''
    args = 'rostopic echo ' + topic + ' > ' + filename
    return subprocess.Popen(args, shell=True)


def main():
    '''
    USAGE: launch_kinefly_experiment.py <driver_name>

    Creates a unique experiment folder with the driver name, launches Kinefly,
    captures flystate data from all three cameras.
    '''
    # create experiment folder from datestamp, driver name?
    DATESTAMP = datestamp()
    experiment_folder = os.path.join(OUTPUT_FOLDER, DATESTAMP + sys.argv[1])
    try:
        os.umask(0002)
        os.mkdir(experiment_folder, 0775)
        os.umask(0022)
    except OSError:
        print 'Cannot create folder %s in %s.' % DATESTAMP, OUTPUT_FOLDER

    # copy kinefly.yaml files to experiment folder
    shutil.copy('kinefly1_pin.yaml', experiment_folder)
    shutil.copy('kinefly2_pin.yaml', experiment_folder)
    shutil.copy('kinefly3_pin.yaml', experiment_folder)

    # launch kinefly and capture video to bagfile
    kinefly_args = 'RIG=kinefly roslaunch Kinefly record.launch'
    kinefly = subprocess.Popen(kinefly_args, shell=True)

    # wait for kinefly to launch completely
    time.sleep(LAUNCH_DELAY)

    # launch processes to capture flystate
    cam1_flystate = rostopic_capture('/kinefly1_pin/flystate',
                                     os.path.join(experiment_folder, 'cam1_flystate.yaml'))
    cam2_flystate = rostopic_capture('/kinefly2_pin/flystate',
                                     os.path.join(experiment_folder, 'cam2_flystate.yaml'))
    cam3_flystate = rostopic_capture('/kinefly3_pin/flystate',
                                     os.path.join(experiment_folder, 'cam3_flystate.yaml'))

    # wait for the experiment to end
    time.sleep(EXPERIMENT_DURATION)

    # end all processes gracefully
    cam1_flystate.terminate()
    cam2_flystate.terminate()
    cam3_flystate.terminate()
    kinefly.terminate()

    # move files to the experiment folder



if __name__ == '__main__':
    main()


