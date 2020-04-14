#! /usr/bin/env python

import rospy 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point,Twist
from math import atan2
from sensor_msgs.msg import LaserScan
import math
import numpy as np

x=0.0
y=0.0
theta=0.0

obstacle_avoiding=False
turtlebot_moving=True

STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR


def get_scan():      #cleaning out laser data
    scan = rospy.wait_for_message('scan', LaserScan)
    
    samples = len(scan.ranges)
    scan_filter=np.asarray(scan.ranges)







    for i in range(samples):
        if scan_filter[i] == float('Inf'):
            scan_filter[i]=3.5
        elif math.isnan(scan_filter[i]):
            scan_filter[i]=0




    return scan_filter,len(scan.ranges)

def newOdom(msg):     #getting x y and theta
    global x
    global y
    global theta

    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y

    rot_q=msg.pose.pose.orientation
    (roll,pitch,theta)=euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])


def lrotate():
    speed.linear.x=0.0
    speed.angular.z=0.5


def rrotate():
    speed.linear.x=0.0
    speed.angular.z=-0.5

def move():
    speed.linear.x=0.2
    speed.angular.z=0.0
    turtlebot_moving=True

def obstacle_move():
    speed.linear.x=0.1
    speed.angular.z=0.0
    turtlebot_moving=True

def stop_bot():
    speed.linear.x=0.0

    turtlebot_moving=False





rospy.init_node("speed_controller")

sub=rospy.Subscriber("/odom",Odometry,newOdom)
pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)

speed=Twist()

r=rospy.Rate(4)

goal=Point()
goal.x=2.5
goal.y=2.5

while not rospy.is_shutdown():
    global turtlebot_moving
    global obstacle_avoiding


    inc_x=goal.x-x
    inc_y=goal.y-y

    angle_to_goal=atan2(inc_y,inc_x)
    lidar_distances,len_data=get_scan()
    min_distance = np.amin(lidar_distances)

    if min_distance<SAFE_STOP_DISTANCE:
        if not obstacle_avoiding:
            stop_bot()
            obstacle_avoiding=True

        elif np.argmin(lidar_distances)>(7*len_data//8) and np.argmin(lidar_distances)<len_data:
            print "rotating left"
            lrotate()

        elif np.argmin(lidar_distances)>0 and np.argmin(lidar_distances)<(len_data//8):
            print "rotating right"
            rrotate()

        else:
            print "move"
            obstacle_move()



    else:
        
        if angle_to_goal-theta>0.1:
            lrotate()

        elif theta-angle_to_goal>0.1:
            rrotate()

        else:
            move()

    pub.publish(speed)
    r.sleep()
