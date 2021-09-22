#!/usr/bin/env python3


## Subscribe to the topic /scan

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from wu0002ie_lab2.msg import scan_range

##Publlisher
closest_point_pub = rospy.Publisher('/closest_point', Float64, queue_size = 1)
furthest_point_pub = rospy.Publisher('/farthest_point', Float64, queue_size = 1)

range_pub = rospy.Publisher('/scan_range', scan_range, queue_size = 1)


def publisher(data):
    closest_point = Float64()
    furthest_point = Float64()

    #create a mask to mask out nan and inf
    mask = np.array(data.ranges)>0.0
    ranges = mask*list(data.ranges)

    closest_point.data = float(min(ranges))
    furthest_point.data = float(max(ranges))

    #set up ranges object
    ranges = scan_range()
    ranges.minimum_range = data.range_min
    ranges.maximum_range = data.range_max
    ranges.header = data.header

    ##print out in terminal (trail and error)
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.header.frame_id)
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s', furthest_point.data)
    closest_point_pub.publish(closest_point)
    furthest_point_pub.publish(furthest_point)
    range_pub.publish(ranges)

    

if __name__ == '__main__':
    try:
        rospy.init_node("ranges_subscribed", anonymous = True)
        rospy.Subscriber('/scan', LaserScan, publisher)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
