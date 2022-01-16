#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

#import for pure pursuit
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

#import for marker
from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
# from std_msgs.msg import Header, ColorRGBA
from utils import *

import numpy as np
from numpy import genfromtxt
from tf.transformations import quaternion_matrix
#import csv log files
#change the path accordingly!!!!!!!!!!!!!!!!!!
my_data = genfromtxt('/home/jayden99/wu0002ie_ws_lab3/src/wu0002ie_lab6/logs/wp-2022-01-15-02-08-00.csv', delimiter = ',')
my_data = my_data[::200,:]
#form homogeneous coordinates
n_data = my_data.shape[0]
h_coor = np.zeros((4,n_data))
waypoint_coor = my_data[:,:2].T #(2, 40773)
h_coor[:2,:] = waypoint_coor
h_coor[3,:] = 1

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        # TODO: create ROS subscribers and publishers.
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.pose_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/nav", AckermannDriveStamped, queue_size=1)
        self.L = 1.1 #1.1
        self.k = 0.5 #0.5        

        self.target_marker_publisher = rospy.Publisher("/target_waypoints", Marker, queue_size=1)

    def pose_callback(self, pose_msg):
        #get the orientation and pos of the car
        quaternion = np.array([pose_msg.pose.pose.orientation.x, 
        pose_msg.pose.pose.orientation.y, 
        pose_msg.pose.pose.orientation.z, 
        pose_msg.pose.pose.orientation.w])
        
        pos = np.array([pose_msg.pose.pose.position.x,
        pose_msg.pose.pose.position.y,
        pose_msg.pose.pose.position.z])

        #form homogeneous transformation matrix
        t_matrix = quaternion_matrix(quaternion)
        t_matrix[:3,3] = pos
        
        #transform the h_coor from map frame to baselink frame
        b_h_coor = np.matmul(np.linalg.inv(t_matrix),h_coor)
        b_coor = b_h_coor[:2,:] #(2,n)
        #filter the points behind the car
        b_coor = b_coor[:,b_coor[0]>0]
        L2_dist = np.sum(b_coor**2,axis = 0)

        target_idx = np.argmin(np.abs(L2_dist - self.L))
        target_L = L2_dist[target_idx]
        target_x = b_coor[0,target_idx]
        target_y = b_coor[1,target_idx]
        target_marker(self.target_marker_publisher,(target_x,target_y))
        #gamma: steering angle
        gamma = self.k*((2*target_y)/(target_L**2))
        #setting speed
        if gamma >0.2:
            speed = 4.8
        elif gamma < -0.2:
            speed = 4.8
        else:
            speed = 6.5
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "nav"
        drive_msg.drive.steering_angle = gamma
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)
    



def main():
    rospy.init_node('pure_pursuit_node')
    #publish marker
    x = waypoint_coor[0,:]
    y = waypoint_coor[1,:]
    coor_for_marker = list(zip(x,y))
    waypoints_marker_publisher = rospy.Publisher("/waypoint_vis", Marker,queue_size=100)
    count = 0
    for i in coor_for_marker:
        waypoints_marker(waypoints_marker_publisher,i,count)
        count += 1
        if count <10:
            rospy.sleep(0.1)
        else:
            rospy.sleep(0.01)
    print("Done plotting waypoints!")
    print("Ready to start navigation!")
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()