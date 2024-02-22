#!/usr/bin/env python3

import rospy
from vision_msgs.msg import Detection2D
from std_msgs.msg import String
import threading
import rosbag
import rospkg

def receive_message():
    # Define the lock
    lock = threading.Lock()

    # Define the path of the package
    pkg_path = rospkg.RosPack().get_path('track_node')
    bag = rosbag.Bag(pkg_path + '/track.bag', 'w')

    def callback(topic, msg):
        with lock:
            bag.write(topic, msg)
            # Print msg info
            rospy.loginfo(f"Msg on Topic {topic} saved to bag")

    def clean_up_bag():
        rospy.loginfo("Cleaning up and closing bag.")
        bag.close()

    rospy.init_node('track_sub_p', anonymous=True)
    rospy.Subscriber('detection_seq_1', Detection2D, lambda msg: callback('/detection_seq_1', msg), queue_size=20)
    rospy.Subscriber('ground_truth_seq_1', Detection2D, lambda msg: callback('/ground_truth_seq_1', msg), queue_size=20)
    rospy.Subscriber('detection_seq_2', Detection2D, lambda msg: callback('/detection_seq_2', msg), queue_size=20)
    rospy.Subscriber('ground_truth_seq_2', Detection2D, lambda msg: callback('/ground_truth_seq_2', msg), queue_size=20)
    rospy.Subscriber('matric_number', String, lambda msg: callback('/matric_number', msg), queue_size=5)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    rospy.on_shutdown(clean_up_bag)

if __name__ == '__main__':
    receive_message()