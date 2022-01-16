from visualization_msgs.msg import Marker
import rospy
from geometry_msgs.msg import Point
def waypoints_marker(publisher,coor,count):
    p_marker = Marker()
    p_marker.header.frame_id = "map"
    p_marker.header.stamp = rospy.get_rostime()
    p_marker.ns = "waypoints"
    p_marker.id = count
    p_marker.type = Marker.SPHERE
    p_marker.action = Marker.ADD
    
    tmp_point = Point()
    tmp_point.x = coor[0]
    tmp_point.y = coor[1]
    tmp_point.z = 1
    p_marker.pose.position = tmp_point

    p_marker.pose.orientation.x = 0
    p_marker.pose.orientation.y = 0
    p_marker.pose.orientation.z = 0
    p_marker.pose.orientation.w = 1

    p_marker.scale.x = 0.2
    p_marker.scale.y = 0.2
    p_marker.scale.z = 0.2

    p_marker.color.r = 0
    p_marker.color.g = 0
    p_marker.color.b = 1
    p_marker.color.a = 1

    p_marker.lifetime = rospy.Duration(0)

    publisher.publish(p_marker)

def target_marker(publisher,coor):
    t_marker = Marker()
    t_marker.header.frame_id = "base_link"
    t_marker.header.stamp = rospy.get_rostime()
    # t_marker.ns = "waypoints"
    # t_marker.id = count
    t_marker.type = Marker.ARROW
    t_marker.action = Marker.ADD
    
    tmp_point = Point()
    tmp_point.x = coor[0]
    tmp_point.y = coor[1]
    tmp_point.z = 1
    t_marker.pose.position = tmp_point

    t_marker.pose.orientation.x = 0
    t_marker.pose.orientation.y = 0
    t_marker.pose.orientation.z = 0
    t_marker.pose.orientation.w = 1

    t_marker.scale.x = 0.5
    t_marker.scale.y = 0.5
    t_marker.scale.z = 0.5

    t_marker.color.r = 1
    t_marker.color.g = 0
    t_marker.color.b = 0
    t_marker.color.a = 1

    t_marker.lifetime = rospy.Duration(0)

    publisher.publish(t_marker)