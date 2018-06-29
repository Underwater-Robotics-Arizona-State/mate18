#!/usr/bin/env python
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs
from sensor_msgs.msg import Image

import cv2
import pickle as pickle
import numpy as np
import filterpy.kalman.kalman_filter as kf

if False:
    from typing import Tuple, List, Any
import math
from scipy.stats import norm

import pubsub, threading
from cv_pubsubs import listen_default


def print_keys_thread():
    """Test thread for getting keystrokes from cv_pubsub"""
    sub_key = pubsub.subscribe("CVKeyStroke")
    sub_cmd = pubsub.subscribe("CVWinCmd")
    msg_cmd = ''
    while msg_cmd != 'quit':
        key_chr = listen_default(sub_key, timeout=.1)  # type: np.ndarray
        if key_chr is not None:
            print("key pressed: " + str(key_chr))
        msg_cmd = listen_default(sub_cmd, block=False, empty='')
    pubsub.publish("CVWinCmd", 'quit')


def start_print_keys_thread():  # type: (...) -> threading.Thread
    t = threading.Thread(target=print_keys_thread, args=())
    t.start()
    return t


import os
import math as m


class SeeingORB(object):
    def __init__(self):
        """Creates an ORB image recognizer that finds all keypoints in an image.
        As well as a FLANN comparator for finding matching keypoints in new images."""
        self.orb = cv2.ORB_create(
            scaleFactor=m.sqrt(m.e),
            nfeatures=1000000,
            scoreType=cv2.ORB_FAST_SCORE
        )  # type: cv2.ORB

        FLANN_INDEX_LSH = 6

        index_params = dict(algorithm=FLANN_INDEX_LSH, trees=5)
        search_params = dict(checks=50)  # or pass empty dictionary
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        self.min_pixels = 1

    def blur_image_radial(self, image):
        # todo: all the implementations on the web are doubly nested for loops. Make that, and a cl version.
        # see: https://stackoverflow.com/a/30102057/782170
        # and: https://stackoverflow.com/a/13699826/782170
        # for implementation ideas
        pass

    def blur_image_for_database(self, image):
        """Blur image so OpenCV doesn't try to recognize webcam noise."""
        return cv2.blur(image, (35, 35))

    def blur_image_for_vision(self, image):
        """Blur image so OpenCV doesn't try to recognize webcam noise."""
        return cv2.blur(image, (35, 35))

    def filter_image(self, image):
        """Return a grayscale image so OpenCV can use it."""
        bw = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # might be useful for text/detail recognition, but throws off ORB
        # bw = cv2.adaptiveThreshold(bw, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        #                           cv2.THRESH_BINARY, 11, 2)

        return bw

    def see_image(self, bw  # type: np.ndarray
                  ):
        """Recognizes all keypoints in an image, on all scales possible, and returns them"""
        image_width, image_height = bw.shape[0], bw.shape[1]
        image_min_dim = min(image_width, image_height)

        self.orb.setNLevels(int(2 * m.log(image_min_dim / self.min_pixels))) # improve recognition on multiple levels

        kp = self.orb.detect(bw)  # type: List[Any]
        kp, des = self.orb.compute(bw, kp)

        return kp, des


class LearningORB(SeeingORB):
    def __init__(self):
        super(LearningORB, self).__init__()

    def study_directory(self, directory, extension='.png'):
        """Create keypoint files for each image in database"""
        for file in os.listdir(directory):
            if file.endswith(extension):
                img = self.filter_image(cv2.imread(directory + '/' + file))
                img = self.blur_image_for_database(img)
                kp, des = self.see_image(img)
                compr = self.compress_sight(kp, des)
                self.write_down_notes(compr, file + '.kp', directory)

    def write_down_notes(self, notes, notes_filename, notes_directory='.'):
        """pickle keypoints and descriptors to a file"""
        pickle.dump(notes, open(notes_directory + os.sep + str(notes_filename), "wb"))

    def compress_sight(self, keypoints, descriptors):
        """Compress recognized keypoints and descriptors to a file"""
        # https://isotope11.com/blog/storing-surf-sift-orb-keypoints-using-opencv-in-python
        i = 0
        temp_array = []
        for point in keypoints:
            temp = (point.pt, point.size, point.angle, point.response, point.octave,
                    point.class_id, descriptors[i])
            i += 1
            temp_array.append(temp)
        return temp_array


class KnowingORB(SeeingORB):

    def __init__(self, name=""):
        """
        Initializes the KnowingORB class with the given name.

        Sets up keypoint lists. Sets up Kalman filters with known good values.
        """
        super(KnowingORB, self).__init__()
        self.kp = []
        self.pos_kp = []
        self.des = []
        self.final_des = None
        self.name = name
        # https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html
        self.kf_n = kf.KalmanFilter(dim_x=2, dim_z=1)
        self.kf_v = kf.KalmanFilter(dim_x=2, dim_z=1)

        self.kf_n.x = self.kf_v.x = np.array([[1.],  # position
                                              [0.]])  # velocity
        self.kf_n.F = self.kf_v.F = np.array([[1., .001],  # .01 defines response rate. .01 is slow.
                                              [0., 1.]])
        self.kf_n.H = self.kf_v.H = np.array([[1., 0.]])
        self.kf_n.P = self.kf_v.P = np.array([[10., 0.],
                                              [0., 10.]])
        self.kf_n.R = self.kf_v.R = 10

        self.__baseline_update = False
        self.__decide_update = False
        self.__baseline_color_update = False
        self.__decide_color_update = False

        if "trap" in self.name:
            self.recog_mean = 3.5
            self.recog_var = 1.8
        elif "tri" in self.name:
            self.recog_mean = 5
            self.recog_var = 2
        else:
            self.recog_mean = 1
            self.recog_var = 1

        self.recog_stats_history = []
        self.decide_stats_history = []
        self.color_stats_history = []
        self.decide_color_stats_history = []
        self.color_mean = 1
        self.color_var = 1

    def read_in_studied_directory(self, directory, extension='.kp'):
        """
        Reads all the keypoints from a directory.

        :param directory: the directory string
        :type directory: str
        :param extension: the extension of our keypoint database files
        :type extension: str
        """
        for file in os.listdir(directory):
            if file.endswith(extension):
                notes = self.__read_notes(directory, file)
                kp, des = self.__notes_to_sight(notes)
                self.kp.extend(kp)
                self.pos_kp.extend([[0, 0] for _ in kp])
                self.des.extend(des)
        self.final_des = np.asarray(self.des, np.uint8)

    def __read_notes(self, directory, file):
        """Unpickles keypoint files from a directory."""
        # https://isotope11.com/blog/storing-surf-sift-orb-keypoints-using-opencv-in-python
        notes = pickle.load(open(str(directory) + os.sep + str(file), "rb"))
        return notes

    def __notes_to_sight(self, array):
        """Gets all keypoints and descriptors from a pickled array"""
        # https://isotope11.com/blog/storing-surf-sift-orb-keypoints-using-opencv-in-python
        keypoints = []
        descriptors = []
        for point in array:
            temp_feature = cv2.KeyPoint(x=point[0][0], y=point[0][1], _size=point[1], _angle=point[2],
                                        _response=point[3], _octave=point[4], _class_id=point[5])
            temp_descriptor = point[6]
            keypoints.append(temp_feature)
            descriptors.append(temp_descriptor)
        return keypoints, np.array(descriptors)

    def recognize_sight(self, kp, des):
        """get all matches between database keypoints and current frame keypoints"""
        if des is None:
            return []
        matches = self.flann.knnMatch(self.final_des,
                                      np.asarray(des, dtype=np.uint8),
                                      k=2)
        return matches

    def recognition_statistics(self, matches, kp):
        """
        Uses Kalman filters to averages the number of database matched keypoints to the current frame
        Keypoints are compared against their previous locations to remove random detection.
        """
        valid_match_count = 0
        average_match_distance = 0

        for m in range(len(matches)):
            if len(matches[m]) != 2:
                continue
            a, b = matches[m]
            aidx = a.trainIdx

            dist = math.sqrt((self.pos_kp[m][0] - kp[aidx].pt[0]) ** 2 + (self.pos_kp[m][1] - kp[aidx].pt[1]) ** 2)

            self.pos_kp[m][:] = kp[aidx].pt[:]
            if dist > 1:
                valid_match_count += 1 / dist
            else:
                valid_match_count += 1
            average_match_distance += b.distance - a.distance

        if len(matches) != 0:
            average_match_distance /= len(matches)

        self.kf_n.predict()
        self.kf_n.update(valid_match_count)

        return self.kf_n.x, self.kf_n.x

    def start_establish_baseline_shape(self):
        self.__baseline_update = True

    def finish_establish_baseline_shape(self):
        self.__baseline_update = False
        self.recog_var = np.var(self.recog_stats_history)
        self.recog_mean = np.mean(self.recog_stats_history)
        self.recog_stats_history = []
        print("{}, new mean[{}], variance[{}%]".format(self.name, self.recog_mean, self.recog_var * 100))

    def start_decide_shape(self):
        self.__decide_update = True

    def finish_decide_shape(self):
        self.__decide_update = False
        decide_var = np.var(self.decide_stats_history)
        decide_mean = np.mean(self.decide_stats_history)
        self.decide_stats_history = []
        print("{}, probability:{}%, variance[{}%]".format(self.name, decide_mean * 100, decide_var * 100))
        return decide_mean, decide_var

    def start_establish_baseline_color(self):
        self.__baseline_color_update = True

    def finish_establish_baseline_color(self):
        self.__baseline_color_update = False
        self.color_var = np.var(self.color_stats_history)
        self.color_mean = np.mean(self.color_stats_history)
        self.color_stats_history = []
        print("{}, baseline hue avg[{}], hue var[{}]".format(self.name, self.color_mean, self.color_var))
        return self.color_mean, self.color_var

    def start_decide_color(self):
        self.__decide_color_update = True

    def finish_decide_color(self):
        self.__decide_color_update = False
        color_var = np.var(self.decide_color_stats_history)
        color_mean = np.mean(self.decide_color_stats_history)
        self.decide_color_stats_history = []
        print("{}, decided hue avg[{}], hue var[{}]".format(self.name, color_mean, color_var))
        print("Please compare hue with self.color_mean and self.color_var for true hue, and compare with known colors.")
        return color_mean, color_var

    def callback(self, frame):
        """
        Main callback function.
        Takes in frame, returns frame showing detection information, and our object's detection probability.

        Gets recognition statistics. Computes the probability that we're seeing our given object.
        That probability is put through a Kalman filter to reduce noise.

        If getting baselines or deciding,
         adds the average keypoint recognition or color statistics to the baseline or decide lists.

        """
        image = self.filter_image(frame)
        image = self.blur_image_for_vision(image)
        kp, des = self.see_image(image)
        matches = self.recognize_sight(kp, des)
        recognition_stats = self.recognition_statistics(matches, kp)

        if self.__baseline_update:
            self.recog_stats_history.append(recognition_stats[0][0])

        d = norm(loc=self.recog_mean, scale=self.recog_var)
        normy = d.cdf(recognition_stats[0][0])
        self.kf_v.predict()
        self.kf_v.update(normy)

        if self.__decide_update:
            self.decide_stats_history.append(self.kf_v.x)

        if self.__baseline_color_update or self.__decide_color_update:
            pass
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            for m in range(len(matches)):
                if len(matches[m]) != 2:
                    continue
                # get keypoint location
                a, b = matches[m]
                aidx = a.trainIdx
                x = kp[aidx].pt[0]
                y = kp[aidx].pt[1]
                # get 10x10 picutre centered around keypoint
                x1 = int(max(x - frame.shape[0] - 5, 0))
                y1 = int(max(y - 5, 0))
                x2 = int(min(x - frame.shape[0] + 5, frame.shape[0]))
                y2 = int(min(y + 5, frame.shape[1]))
                area = hsv[y1:y2, x1:x2]
                # add average color to average color list
                avg_hue = area[:, :, 0].mean()
                # print(avg_hue)
                if self.__baseline_color_update:
                    self.color_stats_history.append(avg_hue)
                else:
                    self.decide_color_stats_history.append(avg_hue)

        return self.draw_recognized(image, matches, image, True, self.kp, kp), self.kf_v.x[0]

    # todo: loop through images in directories and draw recognized keypoints
    def draw_recognized(self, image, matches, image_original=None, draw_keypoints=False, kp=None, kp_store=None):
        """Draws the matched keypoints. Useful for debugging."""

        matches_mask = [[0, 0] for _ in range(len(matches))]

        for m in range(len(matches)):
            if len(matches[m]) != 2:
                continue
            a, b = matches[m]
            if a.distance < 0.8 * b.distance:
                if draw_keypoints:
                    pt1 = (int(kp[m].pt[0]), int(kp[m].pt[1]))
                    pt2 = (int(kp[m].pt[0] + math.cos(math.radians(kp[m].angle)) * kp[m].size),
                           int(kp[m].pt[1] + math.sin(math.radians(kp[m].angle)) * kp[m].size))
                    match_amount = min(((b.distance - a.distance) / float(b.distance)) * 3.0, 1.0)
                    image = cv2.arrowedLine(image, pt1, pt2, (
                        int(kp[m].response * 255), int((match_amount) * 255), int((1 - match_amount) * 255)), 1)
                matches_mask[m] = [1, 0]

        draw_params = dict(matchColor=(0, 255, 255),
                           singlePointColor=(0, 0, 255),
                           matchesMask=matches_mask,
                           flags=0)

        if image_original == None:
            image_original = image

        if draw_keypoints:
            image = cv2.drawMatchesKnn(image, kp, image_original, kp_store, matches, None, **draw_params)

        return (image,)


from cv_pubsubs import webcam_pub as camp
from cv_pubsubs import window_sub as win

if False:
    from typing import Tuple


def get_recognizer_thread(cam):
    """
    Creates a OpenCV window and thread to give OpenCV frames to classes.

    :param cam: Input Camera
    :param knorb1: Knowing ORB/ Image recognizer 1
    :param knorb2: Knowing ORB/ Image recognizer 2
    :param request_size: request camera to start with this resolution
    :param fps_limit: limit fps
    :return: camera thread, for joining and ending program
    """
    def cam_handler(frame, cam_id):
        win.SubscriberWindows.frame_dict[str(cam_id) + "Frame"] = frame

    cam_thread = camp.frame_handler_thread(cam, cam_handler, fps_limit=30,
                                           high_speed=False)

    def combined_callback(frame):
        tempFrame = np.copy(frame)
        tempFrame, tri_prob = tri_knorb.callback(tempFrame)
        frame, trap_prob = trap_knorb.callback(frame)
        out = np.concatenate((tempFrame[0], frame[0]), axis=0) # combine class debug output so we can see everything
        out = cv2.resize(out, (0, 0), fx=0.5, fy=0.5) # resize, as it'd be too big for viewing otherwise

        # debug log output
        if tri_prob > trap_prob:
            print("TRI!")
        elif trap_prob > tri_prob:
            print("TRAP!")

        return [out]

    #run the OpenCV window. Has to be run in the main loop
    win_thread = win.SubscriberWindows(window_names=['recognition'],
                                       input_cams=[cam],
                                       input_vid_global_names=[str(cam) + 'Frame'],
                                       callbacks=[combined_callback]
                                       ).loop()

    return cam_thread


my_dir = os.path.dirname(os.path.abspath(__file__))


def train_thread(tri_knorb, trap_knorb):
    """Establishes all baselines"""
    tri_knorb.start_establish_baseline_shape()
    tri_knorb.start_establish_baseline_color()
    trap_knorb.start_establish_baseline_shape()
    trap_knorb.start_establish_baseline_color()
    for i in range(10):
        print("{} seconds until baseline established".format(10 - i))
        time.sleep(1)
    tri_knorb.finish_establish_baseline_shape()
    tri_knorb.finish_establish_baseline_color()
    trap_knorb.finish_establish_baseline_shape()
    trap_knorb.finish_establish_baseline_color()


def decide_thread(tri_knorb, trap_knorb, prev_thread):
    """Determines probabilities of shape/color detection. Can be compared to each other to find winner."""
    if prev_thread is not None:
        prev_thread.join()

    tri_knorb.start_decide_shape()
    trap_knorb.start_decide_shape()
    tri_knorb.start_decide_color()
    trap_knorb.start_decide_color()
    for i in range(10):
        print("{} seconds until shapes decided".format(10 - i))
        time.sleep(1)
    tri_knorb.finish_decide_shape()
    trap_knorb.finish_decide_shape()
    tri_knorb.finish_decide_color()
    trap_knorb.finish_decide_color()

def img_to_cv(img):
    bridge = CvBridge()
    cv_img = bridge.imgmsg_to_cv2(img)
    get_recognizer_thread(cv_img)

if __name__ == '__main__':
    import time
    import threading

    rospy.init_node('irs', anonymous=False)
    t = rospy.Subscriber("/FrontCamRight/image_raw", Image, img_to_cv)

    if True: # Get keypoints from images in database
        lorb = LearningORB()
        lorb.study_directory(my_dir + os.sep + 'triangle_dataset')
        lorb = LearningORB()
        lorb.study_directory(my_dir + os.sep + 'trapzoid_dataset')

    if True: # determine if images in database are in our webcam's view
        tri_knorb = KnowingORB("triangle")
        tri_knorb.read_in_studied_directory(my_dir + os.sep + 'triangle_dataset')
        trap_knorb = KnowingORB("\t\t\t\t\t\t\t\ttrapezoid")
        trap_knorb.read_in_studied_directory(my_dir + os.sep + 'trapzoid_dataset')
        train_t = threading.Thread(target=train_thread, args=(tri_knorb, trap_knorb))
        train_t.start()
        # train_t = None
        decide_t = threading.Thread(target=decide_thread, args=(tri_knorb, trap_knorb, train_t))
        decide_t.start()
        #t = get_recognizer_thread(0, tri_knorb, trap_knorb)

        tri_knorb.start_decide_shape()
        tri_knorb.finish_decide_shape()

        t.join()
        decide_t.join()
