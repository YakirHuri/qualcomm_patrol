#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import tf

def callback(point):

    quaternion = (point.pose.orientation.x,point.pose.orientation.y,point.pose.orientation.z,point.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

    print(" - "+str(point.pose.position.x)+","+str(point.pose.position.y)+","+str(yaw))
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('waypoints_print_node', anonymous=True)

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)

    print "==================================================="
    print "==================================================="
    print "waypoints:"

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
