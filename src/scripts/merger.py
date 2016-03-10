#!/usr/bin/env python

import rospy
import message_filters
import tf

from tf_message_filter import TfMessageFilter
from sensor_msgs.msg import LaserScan
from tiny_slam.msg import ScanWithPose

def laserScanCallback(laser_scan, transform, target_topic):
    def extractYaw(q):
        #http://answers.ros.org/question/69754/quaternion-transformations-in-python/
        return tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
    
    pose, q = transform
    msg = ScanWithPose()
    msg.scan = laser_scan
    msg.position.x = pose.x
    msg.position.y = pose.y
    msg.position.theta = extractYaw(q)
    target_topic.publish(msg)

if __name__ == '__main__':
    rospy.init_node('data_merger', anonymous=True, log_level=rospy.INFO)

    target_frame = rospy.get_param('~target_frame', 'base_footprint')
    base_frame = rospy.get_param('~base_frame', 'odom_combined')

    pub = rospy.Publisher("/scan_with_pose", ScanWithPose, queue_size=1000)

    ls = message_filters.Subscriber("/laser_scan", LaserScan, queue_size=1000)
    ts = TfMessageFilter(ls, base_frame, target_frame, queue_size=1000, tf_buf_sz=20.0)
    ts.registerCallback(lambda s, t: laserScanCallback(s, t, pub))
    
    rospy.spin()
