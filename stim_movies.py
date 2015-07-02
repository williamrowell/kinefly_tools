#!/usr/bin/python

import sys
import os
import yaml
import h5py
import cv2
import rosbag

import numpy as np
import matplotlib.animation as animation

from matplotlib import pylab, mlab, pyplot
from matplotlib.patches import Ellipse, Wedge
from pylab import *

plt = pyplot


CAM_HEIGHT = 494 # height of raw image in pixels
CAM_WIDTH = 659  # width of raw image in pixels
STIM_CHANNEL = 2 # index of raw stimulus channel
STIM_DIFF_THRESHOLD = 0.1 # threshold for stimulus diff
PRE_STIM = 1  # seconds of video before stimulus
POST_STIM = 2 # seconds of video after stimulus
INTER_STIM = 200 # number of timestamps between stimulus events
FRAME_INTERVAL = 0.03 # time interval between frames in seconds
PLAYBACK_FRAME_RATE = 33  # playback frame rate of output video
#input_path_root = '/home/kineflyjf/bagfiles/'
#output_path_root ='/home/kineflyjf/hdf5files/'
#movie_path_root = '/home/kineflyjf/avifiles/'


def load_yaml(yaml_file):
    """
    Given a dump of all kinefly parameters from ros (yaml_file), produce a dict
    of key->value pairs.
    """
    with open(yaml_file, 'r') as myfile:
        data=myfile.read()
    return yaml.load(data)


def get_topics_len(msgs):
    """
    get_topics_len(msgs) -> dict

    Return a dictionary containing the length of each ros topic with the topic name as the key.
    """
    msg_topics = [x[0] for x in msgs]
    topic_names = set(msg_topics)
    topic_dict = dict()
    for topic_name in topic_names:
        topic_dict.update({topic_name: len([x for x in msg_topics if x == topic_name])})
    return topic_dict


def plot_at_time(time):
    """
    Plot the three fly images at a given time

    Find timestamps greater than the time and use these as the index for the time
    Callback for updating movie figures.
    """
    c1_idx = np.where(c1_ts > time)[0][0]
    c2_idx = np.where(c2_ts > time)[0][0]
    c3_idx = np.where(c3_ts > time)[0][0]
    ai_idx = np.where(ai_ts > time)[0][0]
    im1.set_array(cam1_pixels[c1_idx])
    plot_rois(im1, 'kinefly1_pin', yaml_dict)
    im2.set_array(cam2_pixels[c2_idx])
    plot_rois(im2, 'kinefly2_pin', yaml_dict)
    im3.set_array(cam3_pixels[c3_idx])
    plot_rois(im3, 'kinefly3_pin', yaml_dict)
    if ai_data[ai_idx, STIM_CHANNEL] > STIM_DIFF_THRESHOLD:
        left_txt.set_text('*')
        center_txt.set_text('*')
        right_txt.set_text('*')
    else:
        left_txt.set_text('')
        center_txt.set_text('')
        right_txt.set_text('')
    if not(mod(time, 1)):
        print time
    return [im1, im2, im3]


def plot_rois(canvas, kinefly_pin, yaml_dict):
    """
    Given a yaml_dict with ROI descriptors, draw the kinefly ROIs on canvas.
    """
    # draw left wing
    roi = yaml_dict[kinefly_pin]['gui']['left']
    left = Wedge(center=[roi['hinge']['x'], roi['hinge']['y']],
                 r=roi['radius_inner'],
                 width=roi['radius_outer']-roi['radius_inner'],
                 theta1=np.degrees(roi['angle_lo']),
                 theta2=np.degrees(roi['angle_hi']),
                 fill=False,
                 ec='g')
    canvas.add_artist(left)

    # draw right wing
    roi = yaml_dict[kinefly_pin]['gui']['right']
    right = Wedge(center=[roi['hinge']['x'], roi['hinge']['y']],
                  r=roi['radius_inner'],
                  width=roi['radius_outer']-roi['radius_inner'],
                  theta1=np.degrees(roi['angle_lo']),
                  theta2=np.degrees(roi['angle_hi']),
                  fill=False,
                  ec='r')
    canvas.add_artist(right)

    # draw head
    roi = yaml_dict[kinefly_pin]['gui']['head']
    head = Wedge(center=[roi['hinge']['x'], roi['hinge']['y']],
                 r=roi['radius_inner'],
                 width=roi['radius_outer']-roi['radius_inner'],
                 theta1=np.degrees(roi['angle_lo']),
                 theta2=np.degrees(roi['angle_hi']),
                 fill=False,
                 ec='c')
    canvas.add_artist(head)

    # draw abdomen
    roi = yaml_dict[kinefly_pin]['gui']['abdomen']
    abd = Wedge(center=[roi['hinge']['x'], roi['hinge']['y']],
                r=roi['radius_inner'],
                width=roi['radius_outer']-roi['radius_inner'],
                theta1=np.degrees(roi['angle_lo']),
                theta2=np.degrees(roi['angle_hi']),
                fill=False,
                ec='m')
    canvas.add_artist(abd)

    # draw aux roi
    roi = yaml_dict[kinefly_pin]['gui']['aux']
    aux = Ellipse(xy=[roi['hinge']['x'], roi['hinge']['y']],
                  width=roi['radius1'],
                  height=roi['radius2'],
                  angle=np.degrees(roi['angle']),
                  fill=False,
                  ec='y')
    canvas.add_artist(aux)


# Set file paths and file names
exp_folders = sys.argv[1:]

START_DIR = os.getcwd()

for exp_folder in exp_folders:
    print exp_folder
    os.chdir(exp_folder)
    base_name = exp_folder
    input_file_name = base_name + '.bag'
    output_file_name = base_name + '.hdf5'
    movie_base_name = base_name

    # Load the yaml file
    yaml_dict = load_yaml('rosparam.yaml')

    # Load bagfiles and create a list of messages
    bag = rosbag.Bag(input_file_name)
    msgs = [(topic, msg, t) for topic, msg, t in bag.read_messages()]

    # Get the lengths (# samples) in each topic to determine shape of data
    topic_lengths = get_topics_len(msgs)

    # Open the hdf5 file for writing
    hdf5_file = h5py.File(output_file_name, 'w')

    # Load the stimulus data into the hdf5 file
    ai_shape = (topic_lengths['/stimulus/ai'], 3)
    ai_data = hdf5_file.create_dataset('ai_data', ai_shape, dtype='float64')
    ai_tstamps = hdf5_file.create_dataset('ai_tstamps', ai_shape, dtype='float64')

    # Load the camera1 data into the hdf5 file
    pixels_shape = (topic_lengths['/camera1/image_raw/compressed'], CAM_HEIGHT, CAM_WIDTH)
    cam1_pixels = hdf5_file.create_dataset('cam1_pixels', pixels_shape, dtype='uint8')
    cam1_tstamps = hdf5_file.create_dataset('cam1_tstamps', (pixels_shape[0], 1), dtype='float64')

    # Load the camera2 data into the hdf5 file
    pixels_shape = (topic_lengths['/camera2/image_raw/compressed'], CAM_HEIGHT, CAM_WIDTH)
    cam2_pixels = hdf5_file.create_dataset('cam2_pixels', pixels_shape, dtype='uint8')
    cam2_tstamps = hdf5_file.create_dataset('cam2_tstamps', (pixels_shape[0], 1), dtype='float64')

    # Load the camera3 data into the hdf5 file
    pixels_shape = (topic_lengths['/camera3/image_raw/compressed'], CAM_HEIGHT, CAM_WIDTH)
    cam3_pixels = hdf5_file.create_dataset('cam3_pixels', pixels_shape, dtype='uint8')
    cam3_tstamps = hdf5_file.create_dataset('cam3_tstamps', (pixels_shape[0], 1), dtype='float64')

    # Load ai data into memory
    ai_count = 0
    c1_count = 0
    c2_count = 0
    c3_count = 0
    for msg in msgs:
        if msg[0] == '/stimulus/ai':
            tstamp = msg[1].header.stamp.to_sec()
            ai_data[ai_count] = np.array(msg[1].voltages[:]).astype('float')
            ai_tstamps[ai_count] = tstamp
            ai_count += 1
            if not(np.mod(ai_count, 20000)):
                print ai_count
        elif msg[0] == '/camera1/image_raw/compressed':
            try:
                tstamp = msg[1].header.stamp.to_sec()
                pngdata = np.array(msg[1].data, 'c').view(np.uint8)
                pixels = cv2.imdecode(pngdata, flags=cv2.CV_LOAD_IMAGE_UNCHANGED)
                cam1_pixels[c1_count, :, :] = pixels
                cam1_tstamps[c1_count] = tstamp
                c1_count += 1
            except TypeError:
                print np.shape(pixels)
            if not(np.mod(c1_count, 20000)):
                print c1_count
        elif msg[0] == '/camera2/image_raw/compressed':
            try:
                tstamp = msg[1].header.stamp.to_sec()
                pngdata = np.array(msg[1].data, 'c').view(np.uint8)
                pixels = cv2.imdecode(pngdata, flags=cv2.CV_LOAD_IMAGE_UNCHANGED)
                cam2_pixels[c2_count, :, :] = pixels
                cam2_tstamps[c2_count] = tstamp
                c2_count += 1
            except TypeError:
                print np.shape(pixels)
            if not(np.mod(c2_count, 20000)):
                print c2_count
        elif msg[0] == '/camera3/image_raw/compressed':
            try:
                tstamp = msg[1].header.stamp.to_sec()
                pngdata = np.array(msg[1].data, 'c').view(np.uint8)
                pixels = cv2.imdecode(pngdata, flags=cv2.CV_LOAD_IMAGE_UNCHANGED)
                cam3_pixels[c3_count, :, :] = pixels
                cam3_tstamps[c3_count] = tstamp
                c3_count += 1
            except TypeError:
                print np.shape(pixels)
            if not(np.mod(c3_count, 20000)):
                print c3_count

    # Ensure all data has been flushed to disk and close the file.
    hdf5_file.flush()

    # Set all timepoints relative to t0
    c1_ts = cam1_tstamps - cam1_tstamps[0]
    c2_ts = cam2_tstamps - cam2_tstamps[0]
    c3_ts = cam3_tstamps - cam3_tstamps[0]
    ai_ts = ai_tstamps - ai_tstamps[0]

    stim_indices = np.where(np.diff(ai_data[:, STIM_CHANNEL]) > STIM_DIFF_THRESHOLD)[0]
    stim_counter = 1
    for index, stim_index in enumerate(stim_indices):
        if index > 0 and stim_index - stim_indices[index-1] < INTER_STIM:
            continue

        idx_start = float(ai_ts[stim_index, STIM_CHANNEL]) - PRE_STIM
        idx_stop = float(ai_ts[stim_index, STIM_CHANNEL]) + POST_STIM

        # Fill in the interesting times below:
        # frames = np.arange(start, stop, interval)
        # Leaving the interval at 0.03 means roughly 33 fps
        frames = np.arange(idx_start, idx_stop, FRAME_INTERVAL)

        # Set up the figure for the movie
        fig1 = figure(figsize=(12, 3))
        subplot(1, 3, 1)
        im1 = imshow(cam1_pixels[0], cmap=cm.gray)
        gca().set_xticklabels([])
        gca().set_yticklabels([])
        left_txt = gca().text(60, 60, '', fontsize=20, color='w')

        subplot(1, 3, 2)
        im2 = imshow(cam2_pixels[0], cmap=cm.gray)
        gca().set_xticklabels([])
        gca().set_yticklabels([])
        center_txt = gca().text(60, 60, '', fontsize=20, color='w')

        subplot(1, 3, 3)
        im3 = imshow(cam3_pixels[0], cmap=cm.gray)
        gca().set_xticklabels([])
        gca().set_yticklabels([])
        right_txt = gca().text(60, 60, '', fontsize=20, color='w')

        subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)

        # Create a writer object and record the movie.
        ani = animation.FuncAnimation(fig1, plot_at_time, frames=frames)

        # records the movie as an uncompressed AVI
        movie_file_name = movie_base_name + '_' + 'stim' + str(stim_counter) + '.avi'
        ani.save(movie_file_name, fps=PLAYBACK_FRAME_RATE, extra_args=['-vcodec', 'rawvideo', '-b', '5000k'])

        stim_counter += 1

    # Close the HDF5 file
    hdf5_file.close()

    # Remove HDF5 file
    os.remove(output_file_name)

    os.chdir(START_DIR)
