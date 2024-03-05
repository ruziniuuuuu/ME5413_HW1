#!/usr/bin/env python3

import rospy
from vision_msgs.msg import Detection2D
from std_msgs.msg import String
import cv2
import rosbag
import rospkg
import numpy as np
from cv_bridge import CvBridge
import rosbag

def load_tracks(gt_path):
    # Load ground truth data from file
    # Expected format: [ (x, y, w, h), ...] for each frame
    tracks = []
    with open(gt_path, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            tracks.append((int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])))
    return tracks

def load_firsttrack(firsttrack_txt):
    # Load the first frame's bounding box from file
    with open(firsttrack_txt, 'r') as file:
        line = file.readline().strip()
        x, y, w, h = map(int, line.split(','))
    return x, y, w, h

def publish_matric_number(string_msg):
    # Publish your matriculation number (std_msgs/String)
    matric_number_msg = String()
    matric_number_msg.data = string_msg
    matric_number_pub = rospy.Publisher('matric_number', String, queue_size=10)
    matric_number_pub.publish(matric_number_msg)
    rospy.loginfo("Published matriculation number: %s", string_msg)

def kalman_filter(pkg_path, bag_rel_path):
    bag_path = pkg_path + bag_rel_path
    bag = rosbag.Bag(bag_path, 'r')
    bagMessages = bag.read_messages('/me5413/image_raw')
    br = CvBridge()
    # Init the Kalman filter
    kf = cv2.KalmanFilter(4, 2)

    # Transition matrix
    kf.transitionMatrix = np.array([[1, 0, 0.1, 0],
                                    [0, 1, 0, 0.1],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]], dtype=np.float32)

    # Process noise covariance matrix
    kf.processNoiseCov = np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 0.01, 0],
                                   [0, 0, 0, 0.01]], dtype=np.float32)

    # Measurement matrix
    kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0]], dtype=np.float32)

    # Measurement noise covariance matrix
    kf.measurementNoiseCov = np.array([[1, 0],
                                       [0, 1]], dtype=np.float32)
    
    # Define the search region
    search_region = []

    # Define the template matching method
    if bag_rel_path == '/data/seq_1/seq_1.bag':    
        x, y, w, h = 173, 294, 121, 190
        gts = load_tracks(pkg_path + '/data/seq_1/groundtruth.txt')
        matching_method = cv2.TM_CCOEFF_NORMED
        search_region = [1, 4, 12, 1]
        kf.measurementNoiseCov = np.array([[0.001, 0],
                                           [0, 0.001]], dtype=np.float32)
        
        # Define the publisher
        det_topic = 'detection_seq_1'
        gt_topic = 'ground_truth_seq_1'
    elif bag_rel_path == '/data/seq_2/seq_2.bag':
        x, y, w, h = 240, 25, 110, 315
        gts = load_tracks(pkg_path + '/data/seq_2/groundtruth.txt')
        matching_method = cv2.TM_SQDIFF_NORMED
        search_region = [2, 1, 1, 1]
        kf.measurementNoiseCov = np.array([[1, 0],
                                           [0, 1]], dtype=np.float32) * 10
        
        # Define the publisher
        det_topic = 'detection_seq_2'
        gt_topic = 'ground_truth_seq_2'
    else: # if not bag_1 or bag_1, then error
        rospy.loginfo("Invalid bag file")
        return
    
    pub_det = rospy.Publisher(det_topic, Detection2D, queue_size=10)
    pub_gt = rospy.Publisher(gt_topic, Detection2D, queue_size=10)

    # Initial state (x, y, v_x, v_y) representing the center of the bounding box and the velocity
    kf.statePre = np.array([x + w/2, y + h/2, 0, 0], dtype=np.float32)

    # Init the list to store the detections
    detections = []

    # Store the previous location
    prev_loc = (x, y)

    # Kalman filter
    rate = rospy.Rate(50)
    for i, (_, msg, _) in enumerate(bagMessages):
        # Convert the ROS Image message to OpenCV image
        image = br.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # Define the template
        if i == 0:
            template = image[y:y+h, x:x+w]
        # Define the region for template matching
        x_region = max(0, prev_loc[0] - search_region[0])
        y_region = max(0, prev_loc[1] - search_region[1])
        w_region = min(image.shape[1], prev_loc[0] + w + search_region[2]) - x_region
        h_region = min(image.shape[0], prev_loc[1] + h + search_region[3]) - y_region

        # Template matching
        res = cv2.matchTemplate(image[y_region:y_region+h_region, x_region:x_region+w_region], template, matching_method)
        _, _, _, max_loc = cv2.minMaxLoc(res)
        top_left = (max_loc[0] + x_region, max_loc[1] + y_region)

        # copy the image
        image_copy = image.copy()

        # Define the measurement
        measurement = np.array([top_left[0] + w/2, top_left[1] + h/2], dtype=np.float32)

        # Update the Kalman Filter
        kf.correct(measurement)

        # Predict the next state
        prediction = kf.predict()
        x_pred, y_pred = int(prediction[0] - w/2), int(prediction[1] - h/2)
        x_pred = max(0, x_pred)
        x_pred = min(image.shape[1] - w, x_pred)
        y_pred = max(0, y_pred)
        y_pred = min(image.shape[0] - h, y_pred)

        # Draw the bounding box
        cv2.rectangle(image_copy, (x_pred, y_pred), (x_pred + w, y_pred + h), (0, 255, 0), 2)

        # Draw the ground truth
        gt_x, gt_y, gt_w, gt_h = gts[i]
        cv2.rectangle(image_copy, (gt_x, gt_y), (gt_x + gt_w, gt_y + gt_h), (0, 0, 255), 2)

        # Display the image
        cv2.imshow("Image window", image_copy)

        # Save the detection
        detections.append([x_pred, y_pred, w, h])

        # update the previous location
        prev_loc = (x_pred, y_pred)

        # define the detection message
        det_msg = Detection2D()
        det_msg.header.stamp = rospy.Time.now()
        det_msg.header.frame_id = str(i + 1)
        det_msg.bbox.size_x = w
        det_msg.bbox.size_y = h
        det_msg.bbox.center.x = x_pred + w/2
        det_msg.bbox.center.y = y_pred + h/2

        # publish the detection
        pub_det.publish(det_msg)
        rospy.loginfo("Published detection: frame_id=%s, center_x=%d, center_y=%d, w=%d, h=%d", det_msg.header.frame_id, det_msg.bbox.center.x, det_msg.bbox.center.y, det_msg.bbox.size_x, det_msg.bbox.size_y)

        rate.sleep()
        # define the ground truth message
        gt_msg = Detection2D()
        gt_msg.header.stamp = rospy.Time(0)
        gt_msg.header.frame_id = str(i + 1)
        gt_msg.bbox.size_x = gt_w
        gt_msg.bbox.size_y = gt_h
        gt_msg.bbox.center.x = gt_x + gt_w/2
        gt_msg.bbox.center.y = gt_y + gt_h/2

        # publish the ground truth
        pub_gt.publish(gt_msg)
        rospy.loginfo("Published ground truth: frame_id=%s, center_x=%d, center_y=%d, w=%d, h=%d", gt_msg.header.frame_id, gt_msg.bbox.center.x, gt_msg.bbox.center.y, gt_msg.bbox.size_x, gt_msg.bbox.size_y)

        rate.sleep()

    bag.close()

if __name__ == '__main__':

    # init node
    rospy.init_node('track_pub_p', anonymous=True)
    # get path of the package
    pkg_path = rospkg.RosPack().get_path('track_node')

    try:
        rate = rospy.Rate(1)
        kalman_filter(pkg_path, '/data/seq_1/seq_1.bag')
        kalman_filter(pkg_path, '/data/seq_2/seq_2.bag')
        while not rospy.is_shutdown():
            publish_matric_number('A0285295N')
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")