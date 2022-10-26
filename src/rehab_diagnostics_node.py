#!/usr/bin/env python

#Adding libraries
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import cv2, cv_bridge
from rehab_msgs.msg import bumper
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

rotation = 0.0
twist =Twist()
def odom_callback(msg):
    global rotation
    rotation = msg.pose.pose.orientation.z

j = [0]
def jointState_callback(msg):
    global j
    j = msg.position


image = [0]
depth_width = 0
def Image_callback(msg):
    global image
    bridge = cv_bridge.CvBridge()
    image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
def depth_callback(msg):
    global depth_width
    depth_width = msg.width


l = [0]
def scan_callback(msg):
    global l
    l = msg.ranges

def bumper_callback(msg):
    global id_, state
    id_ = msg.sensor_id
    state = msg.sensor_state
    bumper_status()

rospy.init_node('Rehab_Diagnostic_Node', anonymous=True)
rospy.Subscriber("/scan", LaserScan, scan_callback)
rospy.Subscriber("/camera/color/image_raw", Image, Image_callback)
rospy.Subscriber("/camera/depth_registered/points", PointCloud2, depth_callback)
rospy.Subscriber("/bumper", bumper, bumper_callback)
rospy.Subscriber("/joint_states", JointState, jointState_callback)
rospy.Subscriber("/mobile_base_controller/odom", Odometry, odom_callback)
cmd_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

def camera_status():
    global image
    if len(image)>1:
        print("Rgb Camera: is Okay")
        image = [0]
    else:
        print("Rgb Camera: not okay")

def depth_status():
    global depth_width
    if depth_width>0:
        print("depth Camera: is Okay")
        depth_width = 0
    else:
        print("depth Camera:  not okay")

def lidar_status():
    global l
    if len(l)>1:
        print("Lidar: is Okay")
        l = [0]
    else:
        print("Lidar: not okay")

def bumper_status():
    global id_, state
    print("Bumper: is Okay")


def jointState_status():
    global j
    if len(j)>1:
        print("jointState: is Okay")
        j = [0]
    else:
        print("jointState: not okay")

previous_z = round(rotation,2)
def odom_status():
    global rotation
    for i in range(20):
        twist.angular.z = 0.2
        cmd_pub.publish(twist)
        rospy.sleep(0.1)
    if round(rotation,2)!=previous_z:
        print("Odom: is Okay")
    else:
        print("Odom: not Okay")
        print("Please check the STM connection")
    for i in range(20):
        twist.angular.z = -0.2
        cmd_pub.publish(twist)
        rospy.sleep(0.1)
    print(" ")

print(" ")
print("press any of the bumper")
print(" ")
rospy.sleep(4)
odom_status()

while not rospy.is_shutdown():
    lidar_status()
    camera_status()
    depth_status()
    jointState_status()
    print("---------------------")
    rospy.sleep(3)
