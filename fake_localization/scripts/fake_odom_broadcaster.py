#!/usr/bin/python

#
# Similar to static_transform_broadcaster, this node constantly publishes
# static odometry information (Odometry msg and tf). This can be used
# with fake_localization to evaluate planning algorithms without running
# an actual robot with odometry or localization
#
# Author: Armin Hornung
# License: BSD

import rospy
import tf
from geometry_msgs.msg import Pose, Quaternion, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class fakeAmclPose:
    def __init__(self):
        rospy.Subscriber('/odom', Odometry, self.subOdometry)
        self.base_frame_id = rospy.get_param("~base_frame_id", "/odom")
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "/map")
        self.publish_frequency = rospy.get_param("~publish_frequency", 10000.0)
        self.amcl_pub = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped,queue_size=10)
        self.tf_pub = tf.TransformBroadcaster()
        self.odom = Odometry()

    def subOdometry(self, msg):
        self.odom = msg
        self.publishOdom()

    def publishOdom(self):
        amcl_pose = PoseWithCovarianceStamped()
        amcl_pose.header.frame_id = self.odom_frame_id
        amcl_pose.header.stamp = rospy.Time.now()

        point = Point(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z)
        quat = self.odom.pose.pose.orientation
        amcl_pose.pose.pose = Pose(point,quat)
        # rospy.loginfo("Publishing static odometry from \"%s\" to \"%s\"", self.odom_frame_id, self.base_frame_id)
    
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)


        self.tf_pub.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0),
                        self.odom.header.stamp, self.base_frame_id, self.odom_frame_id)
        self.amcl_pub.publish(amcl_pose)
        

if __name__ == '__main__':
    rospy.init_node('fake_odom')
    node = fakeAmclPose()
    rospy.spin()