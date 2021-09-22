#!/usr/bin/env python3
import rospy
# TODO: import ROS msg types and libraries
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import math
import message_filters

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        
        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # Publishers and Subscribers
        self.pub_brake = rospy.Publisher('/brake', AckermannDriveStamped, queue_size = 1)
        self.pub_brake_bool = rospy.Publisher('/brake_bool', Bool, queue_size = 1)
        self.subs_odom = message_filters.Subscriber('/odom', Odometry)
        self.subs_laser = message_filters.Subscriber('/scan',LaserScan)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.subs_odom, self.subs_laser],1, 0.1)
        self.ts.registerCallback(self.callback)

    def callback(self, odom_msg, scan_msg):
        #data processing
        self.filter_data(odom_msg,scan_msg)
        #call callback function
        self.odom_callback(odom_msg)
        self.scan_callback(scan_msg)

    def odom_callback(self, odom_msg):
        # update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # calculate TTC
        if self.speed <-0.01 or self.speed >0.01:
            if len(self.fdata_array>0):
                ttc = self.fdata_array/self.fvel_array
                self.min_distance = min(0.45*abs(self.speed),0.7)
                min_ttc = min(ttc)
                pos = np.where(ttc == min_ttc)
                min_ttc_beam_angle = self.fangle_list[pos[0][0]]
                threshold = self.min_distance/(self.speed*math.cos(min_ttc_beam_angle))
                #publish braking message
                if min_ttc < threshold:
                    print("BRAKE!!!")
                    speed = AckermannDriveStamped()
                    speed.drive.speed = 0.0
                    braking = Bool()
                    braking.data = True
                    self.pub_brake.publish(speed)
                    self.pub_brake_bool.publish(braking)
                    return

    def filter_data(self,odom_msg,scan_msg):
        # list of filtered data, velocity and angle
        self.fdata_list = []
        self.fvel_list = []
        self.fangle_list = []
        # laser scan values
        range_min = scan_msg.range_min
        range_max = scan_msg.range_max
        ranges = scan_msg.ranges
        angle = -3.142
        angle_increment = scan_msg.angle_increment
        #data preprocessing
        if self.speed != 0:
            for count,dist in enumerate(ranges):
                if dist < range_min or dist > range_max or np.isnan(dist) or np.isinf(dist):
                    angle += angle_increment
                    continue
                else:
                    denominator = max(self.speed*math.cos(angle),0)
                    if denominator == 0:
                        angle += angle_increment
                        continue
                    else:
                        #rear
                        if angle < -2.706 or angle > 2.706:
                            adj_dist = dist + (0.28)*math.cos(angle)
                        #front
                        elif angle > -0.436 and angle < 0.436:
                            adj_dist = dist - (0.13)*math.cos(angle)
                        else:
                            adj_dist = dist + (0.1)*abs(math.cos(angle))
                            # adj_dist = dist
                        self.fdata_list.append(adj_dist)
                        self.fvel_list.append(denominator)
                        self.fangle_list.append(angle)
                        angle += angle_increment

        self.fdata_array = np.array(self.fdata_list, dtype = np.float64)
        self.fvel_array = np.array(self.fvel_list, dtype = np.float64)

def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()