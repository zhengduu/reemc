#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from std_msgs.msg import Header

def callback(data):
    p.polygon = data.polygon
    p.header.frame_id = data.header.frame_id

def talker():
    
    pub = rospy.Publisher('/controller/LeftSolePolygon1', PolygonStamped, queue_size=10)
    rospy.Subscriber("/controller/LeftSolePolygon", PolygonStamped, callback)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        p.header.stamp = rospy.Time.now()
        pub.publish(p)
        rate.sleep()

if __name__ == '__main__':
    p = PolygonStamped()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
