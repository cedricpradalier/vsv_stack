#!/usr/bin/env python
import roslib; roslib.load_manifest('roadside_mapper')
import rospy
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float32
from math import pi

class TrackBlackLine:
    def __init__(self):
        self.last_point = -1e10
        self.line_value = None

        rospy.init_node('track_black_line')
        self.z_track = rospy.get_param("~z_track",-0.5)
        self.z_idle = rospy.get_param("~z_idle",0.0)
        self.x_offset = rospy.get_param("~x_offset",0.5)
        self.sub = rospy.Subscriber('~black_line', PointStamped, self.line_cb)
        self.pose_pub = rospy.Publisher('~position_command', Point)
        self.tool_pub = rospy.Publisher('~tool_command', Float32)

    def line_cb(self,value):
        self.last_point = rospy.rostime.get_time()
        self.line_value = value
        self.line_value.point.x += self.x_offset

    def run(self):
        timeout = True
        rate = rospy.Rate(10)
        state = "Ready"
        while not rospy.is_shutdown():
            if (rospy.rostime.get_time() - self.last_point) < 0.5: 
                if timeout:
                    timeout = False
                    rospy.loginfo("Track Black Line: Line found")
                self.tool_pub.publish(Float32(-pi/2))
                self.pose_pub.publish(Point(self.line_value.point.x,0.0,self.z_track))
            else:
                if not timeout:
                    timeout = True
                    rospy.loginfo("Track Black line: Lost line")
                self.tool_pub.publish(Float32(-pi/2))
                if self.line_value:
                    self.pose_pub.publish(Point(self.line_value.point.x,0.0,self.z_idle))
                else:
                    self.pose_pub.publish(Point(2.5,0.0,self.z_idle))
            rate.sleep()


if __name__ == '__main__':
    try:
        teleop = TrackBlackLine()
        teleop.run()
        
    except rospy.ROSInterruptException:
        pass
