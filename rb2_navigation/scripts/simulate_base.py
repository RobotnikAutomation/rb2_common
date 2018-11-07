#!/usr/bin/env python  
import rospy

import tf
import tf_conversions
import geometry_msgs.msg
import std_msgs.msg

from nav_msgs.msg import Odometry

import math

rospy.init_node('simulate_cmdvel')
current_x = 0
current_y = 0
current_theta = 0
ts = 0.1
last_cmd_stamp = rospy.Time.now()
br = tf.TransformBroadcaster()
current_cmd = geometry_msgs.msg.Twist()
enabled_cmd = True

def handle_enabled_cmd(msg):
    global enabled_cmd
    enabled_cmd = msg.data


def handle_cmd_vel(msg):
    global last_cmd_stamp
    global current_cmd

    last_cmd_stamp = rospy.Time.now()
    current_cmd = msg



rospy.Subscriber('~cmd_vel',
    geometry_msgs.msg.Twist,
    handle_cmd_vel,
    queue_size=1)

rospy.Subscriber('~enabled_cmd',
    std_msgs.msg.Bool,
    handle_enabled_cmd,
    queue_size=1)

base_frame_id = rospy.get_param("~base_frame_id", "base_link")
odom_frame_id = rospy.get_param("~odom_frame_id", "odom")

odom = Odometry()
odom.header.frame_id = odom_frame_id
odom.child_frame_id = base_frame_id
pub = rospy.Publisher('~odom', Odometry, queue_size=1)

rate = rospy.Rate(50)
ts = 1.0/50.0
cmd_watchdog = rospy.Duration(0.5)
cmd_watchdog_message_shown = False
rospy.loginfo("running simulate cmdvel")
while not rospy.is_shutdown():
    if rospy.Time.now() - last_cmd_stamp < cmd_watchdog:
        cmd_watchdog_message_shown = False
    else:
        current_cmd = geometry_msgs.msg.Twist()
        if not cmd_watchdog_message_shown:
            rospy.logwarn("cmd watchdog!")
        cmd_watchdog_message_shown = True

    current_x += math.cos(current_theta)*current_cmd.linear.x*ts - math.sin(current_theta)*current_cmd.linear.y*ts
    current_y += math.sin(current_theta)*current_cmd.linear.x*ts + math.cos(current_theta)*current_cmd.linear.y*ts
    current_theta += current_cmd.angular.z*ts

    br.sendTransform((current_x, current_y, 0), tf.transformations.quaternion_from_euler(0, 0, current_theta), rospy.Time.now(), base_frame_id, odom_frame_id)
    odom.pose.pose = geometry_msgs.msg.Pose(geometry_msgs.msg.Point(current_x, current_y, 0), geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, current_theta)))
    odom.twist.twist = current_cmd
    pub.publish(odom)
    rate.sleep()

