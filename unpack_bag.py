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
                 {'head': ['angles', 'radii'],
                  'abdomen': ['angles', 'radii'],
                  'left': ['angles'],
                  'right': ['angles'],
                  'aux': ['intensity']},
             '/kinefly2_pin/flystate':
                 {'left': ['angles', 'radii'],
                  'right': ['angles', 'radii'],
                  'aux': ['intensity']},
             '/kinefly3_pin/flystate':
                 {'left': ['angles', 'radii'],
                  'right': ['angles', 'radii'],
                  'aux': ['intensity']},
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
TOPIC_TYPES = {'/stimulus/ai': 'Kinefly/MsgAnalogIn',
               '/kinefly1_pin/flystate': 'Kinefly/MsgFlystate',
               '/kinefly2_pin/flystate': 'Kinefly/MsgFlystate',
               '/kinefly3_pin/flystate': 'Kinefly/MsgFlystate',
               '/camera1/image_raw/compressed': 'sensor_msgs/CompressedImage',
               '/camera2/image_raw/compressed': 'sensor_msgs/CompressedImage',
               '/camera3/image_raw/compressed': 'sensor_msgs/CompressedImage'}


def get_topic_counts(msgs):
    """
    Given a list of messages, find the topics and count for each topic.
    """
    topic_list = [x[0] for x in msgs]
    topic_set = set(topic_list)
    return {topic:topic_list.count(topic) for topic in topic_set}


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

    if TOPIC_TYPES[my_topic] == 'Kinefly/MsgAnalogIn':
        # create dataset in hdf5_file
        shape = (len(msgs), STIM_CHANNELS)
        data = hdf5_file.create_dataset(name + '_data', shape, dtype='float64')
        tstamps = hdf5_file.create_dataset(name + '_tstamps', shape, dtype='float64')
        # store data in hdf5_file
        for count, msg in enumerate(msgs):
            tstamps[count] = msg[1].header.stamp.to_sec()
            data[count] = np.array(msg[1].voltages[:]).astype('float')

    elif TOPIC_TYPES[my_topic] == 'sensor_msgs/CompressedImage':
        # create dataset in hdf5_file
        shape = (len(msgs), CAM_HEIGHT, CAM_WIDTH)
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

    elif TOPIC_TYPES[my_topic] == 'Kinefly/MsgFlystate':
        # create dataset in hdf5_file
        # only works for fields returning a single value
        shape = (len(msgs), 1)
        tstamps = hdf5_file.create_dataset(name + '_tstamps', shape, dtype='float64')
        data = dict()
        for subtopic in my_subtopics:
            data[subtopic] = dict()
            for field in subtopic:
                data[subtopic][field] = hdf5_file.create_dataset('_'.join([name, subtopic, field]), shape, dtype='float64')
        # store data in hdf5_file
        for count, msg in enumerate(msgs):
            tstamps[count] = msg[1].header.stamp.to_sec()
            for subtopic in my_subtopics:
                for field in my_subtopics[subtopic].keys():
                    data_point = getattr(getattr(msg[1], subtopic), field)
                    if type(data_point) == float:
                        data[subtopic][field][count] = data_point
                    elif type(data_point) == tuple:
                        data[subtopic][field][count] = data_point[0]

    # ensure all data has been flushed to disk
    hdf5_file.flush()


def main():
    bag_file = sys.argv[1]
    base_name = bag_file[:-4]
    input_file_name = bag_file
    output_file_name = base_name + '.hdf5'

    # Load bagfiles and create a list of messages
    bag = rosbag.Bag(input_file_name)

    # Open the hdf5 file for writing
    hdf5_file = h5py.File(output_file_name, 'w')

    msgs = [(topic, msg, t) for topic, msg, t in bag.read_messages()]

    topic_lengths = get_topic_counts(msgs)
    topics = topic_lengths.keys()

    for topic in topics:
        unpack(bag, hdf5_file, topic, TOPIC_NAMES[topic], my_subtopics=SUBTOPICS[topic])

    hdf5_file.flush()
    hdf5_file.close()

if __name__ == '__main__':
    main()
