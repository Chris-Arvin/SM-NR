#!/usr/bin/env python
# -*- coding: utf-8 -*-
from costmap_converter import msg
from genpy import message
import rospy
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from vehicle_simulator.msg import SocialVehicles
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32, QuaternionStamped, Quaternion, TwistWithCovariance
from std_msgs.msg import Header

class Obstacle_Trans:
    def __init__(self):
        self.dynamic_veh_positions = []
        self.is_completed = False
        self.sub_static_polys = rospy.Subscriber("/simulate_social_car/static_sv_polys", MarkerArray, self.staticVehCB, queue_size=3)
        self.sub_dynamic_polys = rospy.Subscriber("/simulate_social_car/dynamic_sv_polys", MarkerArray, self.dynamicVehCB, queue_size=3)
        self.sub_dynamic_positions = rospy.Subscriber("/simulate_social_car/dynamic_sv_states", SocialVehicles, self.dynamicVehPositionCB, queue_size=3)
        self.pub_obs_from_wall = rospy.Publisher('/translated/obstacles_wall', ObstacleArrayMsg, queue_size=1)
        self.pub_obs_from_static_veh = rospy.Publisher('/translated/obstacles_static_veh', ObstacleArrayMsg, queue_size=1)
        self.pub_obs_from_dynamic_veh = rospy.Publisher('/translated/obstacles_dynamic_veh', ObstacleArrayMsg, queue_size=1)
        

    def staticVehCB(self, msg):
        obstacle_msg = ObstacleArrayMsg()
        obstacle_msg.header.stamp = rospy.Time.now()
        obstacle_msg.header.frame_id = "odom"
        i = 0
        for veh in msg.markers:
            veh_footprint = ObstacleMsg()
            veh_footprint.id = i
            i+=1
            for p in veh.points:
                point = Point32()
                point.x = p.x
                point.y = p.y
                veh_footprint.polygon.points.append(point)
            veh_footprint.velocities.twist.linear.x = 0
            veh_footprint.velocities.twist.linear.y = 0
            veh_footprint.velocities.twist.linear.z = 0
            veh_footprint.velocities.twist.angular.x = 0
            veh_footprint.velocities.twist.angular.y = 0
            veh_footprint.velocities.twist.angular.z = 0
            obstacle_msg.obstacles.append(veh_footprint)
        self.pub_obs_from_static_veh.publish(obstacle_msg)

    def dynamicVehPositionCB(self, msg):
        self.is_completed = False
        self.dynamic_veh_positions=[]
        for veh in msg.vehicles:
            self.dynamic_veh_positions.append([veh.twist.twist.linear.x, veh.twist.twist.angular.z])
        self.is_completed = True

    def dynamicVehCB(self, msg):
        if self.is_completed == False:
            return
        dynamic_veh_positions = self.dynamic_veh_positions
        # print(len(dynamic_veh_positions))
        # print(len(msg.markers)/5)
        obstacle_msg = ObstacleArrayMsg()
        obstacle_msg.header.stamp = rospy.Time.now()
        obstacle_msg.header.frame_id = "odom"
        i = -1
        for veh in msg.markers:
            i+=1
            if i%5!=0:
                continue
            veh_footprint = ObstacleMsg()
            veh_footprint.id = i
            for p in veh.points:
                point = Point32()
                point.x = p.x
                point.y = p.y
                veh_footprint.polygon.points.append(point)
            veh_footprint.velocities.twist.linear.x = dynamic_veh_positions[i/5][0]
            veh_footprint.velocities.twist.linear.y = 0
            veh_footprint.velocities.twist.linear.z = 0
            veh_footprint.velocities.twist.angular.x = 0
            veh_footprint.velocities.twist.angular.y = 0
            veh_footprint.velocities.twist.angular.z = dynamic_veh_positions[i/5][1]
            obstacle_msg.obstacles.append(veh_footprint)
        self.pub_obs_from_dynamic_veh.publish(obstacle_msg)

    def pubWall(self):
        # wall1: (-5,-0.4) ~ (5,-0.4)
        # wall2: (-5, 0.4) ~ (5, 0.4)
        obstacle_msg = ObstacleArrayMsg()
        obstacle_msg.header.stamp = rospy.Time.now()
        obstacle_msg.header.frame_id = "odom"
        wall1 = ObstacleMsg()
        wall1.id = 1
        temp_p1 = Point32()
        temp_p1.x = -5
        temp_p1.y = -0.43
        temp_p2 = Point32()
        temp_p2.x = 5
        temp_p2.y = -0.43
        wall1.polygon.points.append(temp_p1)
        wall1.polygon.points.append(temp_p2)

        temp_p3 = Point32()
        temp_p3.x = -5
        temp_p3.y = 0.43
        temp_p4 = Point32()
        temp_p4.x = 5
        temp_p4.y = 0.43
        wall2 = ObstacleMsg()
        wall2.id = 2
        wall2.polygon.points.append(temp_p3)
        wall2.polygon.points.append(temp_p4)

        obstacle_msg.obstacles.append(wall1)
        obstacle_msg.obstacles.append(wall2)
        self.pub_obs_from_wall.publish(obstacle_msg)

if __name__ == '__main__':
    rospy.init_node('dynamic_obstacle_listener', anonymous=True)
    OT = Obstacle_Trans()
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        OT.pubWall()
        rate.sleep()






