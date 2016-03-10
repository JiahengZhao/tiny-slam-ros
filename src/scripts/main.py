#!/usr/bin/env python

import rospy
import tinyslam
import random
import tf2_ros
import tf
import math
import geometry_msgs
import message_filters
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from tf_message_filter import TfMessageFilter
from tiny_slam.msg import ScanWithPose
import cProfile, pstats, StringIO, atexit

def show_profiler_data(pr, fname):
    pr.disable()
    s = StringIO.StringIO()
    sortby = 'tottime'#'cumulative'
    ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    ps.print_stats()
    f = open(fname, "w")
    f.write(s.getvalue())
    f.close()


#global_pr = cProfile.Profile()
#atexit.register(show_profiler_data, global_pr, "profiling")
#global_pr.enable()

slam_map = tinyslam.Map(256, 25)
robot_state = tinyslam.RobotState()
last_robot_pose = tinyslam.RobotPose()

def generate_transform_msg(ch, par, x = 0, y = 0, theta = 0):
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = par
    t.child_frame_id = ch
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0
    q = tf.transformations.quaternion_from_euler(0, 0, theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    return t

def ros2internalScan(data):
    raw_scan = tinyslam.RawLaserScanData()
    raw_scan.max_distance = data.range_max
    raw_scan.angle_min = data.angle_min
    raw_scan.angle_max = data.angle_max
    raw_scan.scan_data = []
    effective_min_range = data.range_min#max(0.27, data.range_min) # fix data set 'noise'
    for r in data.ranges:
        effective_range = 0
        if r < effective_min_range:
            effective_range = -1
        elif effective_min_range <= r < data.range_max:
            effective_range = r
        raw_scan.scan_data.append(effective_range)
    raw_scan.scan_size = len(raw_scan.scan_data)
    global robot_state
    return tinyslam.LaserScan(raw_scan, robot_state, slam_map)


def update_robot_state(robot_state, position):
    global last_robot_pose
    new_robot_pose = tinyslam.RobotPose(position.x, position.y, position.theta)
    d_pose = new_robot_pose - last_robot_pose

    robot_state.pose.translate(d_pose[0], d_pose[1]).rotate(d_pose[2])
    last_robot_pose = new_robot_pose

    br = tf2_ros.TransformBroadcaster()
    br.sendTransform(generate_transform_msg("tinySlam_bot", "odom_combined",
            robot_state.pose.x, robot_state.pose.y, robot_state.pose.theta))
    br.sendTransform(generate_transform_msg("lzr_debug", "odom_combined",
            robot_state.pose.x, robot_state.pose.y, robot_state.pose.theta))


tf_lstnr = None
lzr_debug = None

def handle_scan_with_pose(swp, pr):
    pr.enable()
    raw_laser_scan = swp.scan
    position = swp.position

    global slam_map, robot_state
    update_robot_state(robot_state, position)

    laser_scan = ros2internalScan(raw_laser_scan)
    refined_pose = robot_state.pose

    lzr_debug.publish(raw_laser_scan)
#    global slam_map
#    slam_map = tinyslam.Map(512, 20)
    robot_state.pose = slam_map.localize(robot_state.pose, laser_scan)
    slam_map.update(robot_state.pose, laser_scan, 50 if refined_pose == robot_state.pose else 25)


    br = tf2_ros.TransformBroadcaster()
    br.sendTransform(generate_transform_msg("refined", "odom_combined",
                                            refined_pose.x, refined_pose.y, refined_pose.theta))

    pr.disable()
#    publish_grid_map()
'''
    r = 100
    for x in range(-r, r):
        y = int(math.sqrt(r**2 - x**2))
        for p in tinyslam.line2D(0, 0, x, y):
            slam_map[p] = 0
        for p in tinyslam.line2D(0, 0, x, -y):
            slam_map[p] = 0
'''
            
def odometryCallback(data):
    def extractYaw(q):
        #http://answers.ros.org/question/69754/quaternion-transformations-in-python/
        return tf.transformations.euler_from_quaternion(q)[2]

    def vec3len(vec):
        # TODO: simplify
        return math.sqrt(vec.x**2 + vec.y**2 + vec.z**2)

    last_position = data.pose.pose.position
    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)

    robot_pose = tinyslam.RobotPose(last_position.x, last_position.y, extractYaw(quaternion))
    lin_speed = vec3len(data.twist.twist.linear)
    ang_speed = vec3len(data.twist.twist.angular)

    global robot_state
    robot_state.update(robot_pose, lin_speed, ang_speed)

    br = tf2_ros.TransformBroadcaster()
    br.sendTransform(generate_transform_msg("tinySlam_bot", "odom_combined",
                                            last_position.x, last_position.y, robot_state.pose.theta))
                    
#    global slam_map
#    slam_map[(int(robot_pose.x / slam_map.scale), int(robot_pose.y / slam_map.scale))] = 0
    return


pub = None
def publish_grid_map():
    grid_map = OccupancyGrid()
    grid_map.info.map_load_time = rospy.Time.now()
    grid_map.info.width = slam_map.size
    grid_map.info.height = slam_map.size
    grid_map.info.resolution = slam_map.scale
    grid_map.info.origin.position.x = -slam_map.size / 2 * slam_map.scale
    grid_map.info.origin.position.y = -slam_map.size / 2 * slam_map.scale
    grid_map.info.origin.position.z = 0
    grid_map.data = slam_map.to_ros_grid()
    pub.publish(grid_map)

if __name__ == '__main__':
    rospy.init_node('tinyslam', anonymous=False, log_level=rospy.INFO)
    local_pr = cProfile.Profile()
    atexit.register(show_profiler_data, local_pr, "handler_prof")
    ls = rospy.Subscriber("/scan_with_pose", ScanWithPose,
                          lambda swp: handle_scan_with_pose(swp, local_pr))

    global pub, lzr_debug
    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=100)
    lzr_debug = rospy.Publisher("/lzr_debug", LaserScan, queue_size=100)
    
    rate = rospy.Rate(1.0 / 3)
    while not rospy.is_shutdown():
        publish_grid_map()
        rate.sleep()
