#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 7.5#TODO 8
kd = 0.0005#TODO 0.005
ki = 0.0001#TODO 0.005
servo_offset = 0.0
prev_error = 0.0 
prev_time = 0.0
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.65 #ori = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'
        self.starting_time = rospy.get_time()
        self.prev_time = rospy.get_time()
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)#TODO: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=1) #TODO: Publish to drive

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        ranges = np.array(data.ranges)
        index = int(((angle+90)/360)*len(ranges))
        distance = ranges[index]
        if not(np.isnan(distance)):
            return distance

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global prev_time
        global kp
        global ki
        global kd
        angle = 0.0
        current_time = rospy.get_time()
        dt = current_time - self.prev_time
        integral += prev_error*dt
        #TODO: Use kp, ki & kd to implement a PID controller for 
        angle = kp*error + ki*integral + kd*((error - prev_error)/dt)
        prev_error = error
        self.prev_time = current_time
        #setting velocity
        # angle = -math.radians(angle)
        # print(angle)
        if abs(angle) >= 0 and abs(angle) <= math.radians(10):
            velocity = 1.5
        elif abs(angle) > math.radians(10) and abs(angle) <= math.radians(20):
            vleocity = 1.0
        else:
            velocity = 0.5
        #publishing message to /nav topic
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        theta = 7*(math.pi)/18 #70 degrees
        a_angle_deg = 180 - 70
        b_angle_deg = 180
        a_angle = math.pi - theta
        b_angle = math.pi
        L = 1.95*CAR_LENGTH
        a = self.getRange(data, a_angle_deg)
        b = self.getRange(data, b_angle_deg)
        alpha = math.atan((a*math.cos(theta)-b)/a*math.sin(theta))
        Dt = b*math.cos(alpha)
        
        Dt1 = Dt + L*math.sin(alpha)
        error = Dt1 - leftDist
        print(error)
        return error

    def lidar_callback(self, data):
        """ 
        """
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT) #TODO: replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
