#!/usr/bin/python

"""
@created: 20150430
@edited: 20150430
@author: William Rowell
@contact: william.rowell@gmail.com
"""

import sys
import rosbag
import cv2
import h5py
import numpy as np


STIM_CHANNELS = 3  # number of analog input stimulus channels
CAM_HEIGHT = 494  # height of raw image in pixels
CAM_WIDTH = 659  # width of raw image in pixels
SUBTOPICS = {'/stimulus/ai': None,
             '/kinefly1_pin/flystate':
                 {'head': [],
                  'abdomen': [],
                  'left': [],
                  'right': [],
                  'aux': []},
             '/kinefly2_pin/flystate':
                 {'head': [],
                  'abdomen': [],
                  'left': [],
                  'right': [],
                  'aux': []},
             '/kinefly3_pin/flystate':
                  {'head': [],
                   'abdomen': [],
                   'left': [],
                   'right': [],
                   'aux': []},
             '/camera1/image_raw/compressed': None,
             '/camera2/image_raw/compressed': None,
             '/camera3/image_raw/compressed': None}
TOPIC_NAMES = {'/stimulus/ai': 'ai',
               '/kinefly1_pin/flystate': 'flystate1',
               '/kinefly2_pin/flystate': 'flystate2',
               '/kinefly3_pin/flystate': 'flystate3',
               '/camera1/image_raw/compressed': 'cam1',
               '/camera2/image_raw/compressed': 'cam2',
               '/camera3/image_raw/compressed': 'cam3'}


def unpack(bag, hdf5_file, my_topic, name, my_subtopics=None):
    """
    Unpack messages from a bag file.

    unpack(bag, my_topic, name, my_subtoptics = None) -> None
    @param bag: specify the rosbag
    @type bag: object created with rosbag
    @param hdf5_file: specify the hdf5_file to unpack into
    @type hdf5_file: object created with h5py
    @param my_topic: specify the topic you want to unpack
    @type my_topic: str
    @param name: specify the name that will be used in the hdf5 file
    @type name: str
    @param my_subtopics: subtopics and fields to grab from flystate msgs
    @type my_subtopics: optional dict of subtopics and corresponding
                        lists of fields for each subtopic
    @return None
    """
    msgs = [(topic, msg, t) for topic, msg, t
            in bag.read_messages(topics=my_topic)]

    if types[my_topic] == 'Kinefly/MsgAnalogIn':
        # create dataset in hdf5_file
        shape = (topic_lengths[my_topic], STIM_CHANNELS)
        data = hdf5_file.create_dataset(name + '_data', shape, dtype='float64')
        tstamps = hdf5_file.create_dataset(name + '_tstamps', shape, dtype='float64')
        # store data in hdf5_file
        for count, msg in enumerate(msgs):
            tstamps[count] = msg[1].header.stamp_to_sec()
            data[count] = np.array(msg[1].voltages[:]).astype('float')

    elif types[my_topic] == 'sensor_msgs/CompressedImage':
        # create dataset in hdf5_file
        shape = (topic_lengths[my_topic], CAM_HEIGHT, CAM_WIDTH)
        data = hdf5_file.create_dataset(name + '_pixels', shape, dtype='uint8')
        tstamps = hdf5_file.create_dataset(name + '_tstamps', shape, dtype='float64')
        # store data in hdf5_file
        for count, msg in enumerate(msgs):
            try:
                tstamps[count] = msg[1].header.stamp.to_sec()
                pngdata = np.array(msg[1].data, 'c').view(np.uint8)
                pixels = cv2.imdecode(pngdata, flags=cv2.CV_LOAD_IMAGE_UNCHANGED)
                data[count, :, :] = pixels
            except TypeError:
                print np.shape(pixels)

    elif types[my_topic] == 'Kinefly/MsgFlystate':
        # create dataset in hdf5_file
        # only works for fields returning a single value
        shape = (topic_lengths[my_topic], 1)
        tstamps = hdf5_file.create_dataset(name + '_tstamps', shape, dtype='float64')
        data = dict()
        for subtopic in my_subtopics:
            data[subtopic] = dict()
            for field in subtopic:
                data[subtopic][field] = hdf5_file.create_dataset('_'.join([name, subtopic, field]), shape, dtype='float64')
        # store data in hdf5_file
        for count, msg in enumerate(msgs):
            tstamps[count] = msg[1].header.stamp_to_sec()
            for subtopic in my_subtopics:
                for field in subtopic:
                    data_point = getattr(getattr(msgs[1], subtopic), field)
                    if type(data_point) == float:
                        data[subtopic][field][count] = data_point
                    elif type(data_point) == tuple:
                        data[subtopic][field][count] = data_point[0]

    # ensure all data has been flushed to disk
    hdf5_file.flush()


def main():
    bag_file = sys.argv[1:]
    base_name = bag_file[:-4]
    input_file_name = bag_file
    output_file_name = base_name + '.hdf5'

    # Load bagfiles and create a list of messages
    bag = rosbag.Bag(input_file_name)

    # Open the hdf5 file for writing
    hdf5_file = h5py.File(output_file_name, 'w')

    types_and_topics = bag.get_type_and_topic_info()
    topics = types_and_topics[1].keys()
    types = {topic: types_and_topics[1][topic][0] for topic in topics}
    topic_lengths = {topic: types_and_topics[1][topic][1] for topic in topics}
    global types
    global topic_lengths

    for topic in topics:
        unpack(bag, hdf5_file, topic, TOPIC_NAMES[topic], my_subtopics=SUBTOPICS[topic])

    hdf5_file.flush()
    hdf5_file.close()

if __name__ == '__main__':
    main()