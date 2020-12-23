#!/usr/bin/env python

import math
import rospy
import actionlib
import tf2_ros

from geometry_msgs.msg import Pose, Twist, TransformStamped, Vector3
from simple_rover_move.msg import MoveAction, MoveResult

class SimpleMoveServer:
    def __init__(self):#, xy_tolerance, rot_tolerance):
        self.server = actionlib.SimpleActionServer('move_rover', MoveAction, self.execute, False)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        #self.xy_tolerance = xy_tolerance
        #self.rot_tolerance = rot_tolerance

        self.server.start()

    def execute(self, goal):
        r = MoveResult()
        rate = rospy.Rate(8)
        cmd_vel = Twist()
        dir_x = bool()
        dir_y = bool()
        inited = False
        rospy.loginfo("get_msg")
            #trans = self.tfBuffer.lookup_transform("odom", "base_link", rospy.Time())
            #self.pub.publish(cmd_vel)
        while not rospy.is_shutdown():
                #trans = self.tfBuffer.lookup_transform("odom", "base_link", rospy.Time())
                #try:
            trans = self.tfBuffer.lookup_transform("map", "base_link", rospy.Time())
            now_pose = trans.transform.translation
            if inited == False:
                cmd_vel.linear.y = goal.vel.linear.y#goal.trans_vel*math.cos(theta)
                cmd_vel.linear.x = goal.vel.linear.x#goal.trans_vel*math.sin(theta)

                dist_x = now_pose.x + (goal.dist if cmd_vel.linear.x >= 0 else -goal.dist)
                dir_x = bool(dist_x >= now_pose.x)
                dist_y = now_pose.y + (goal.dist if cmd_vel.linear.y >= 0 else -goal.dist)
                dir_y = bool(dist_y >= now_pose.y)
                self.pub.publish(cmd_vel)
                inited = True

                #except:
                #    rospy.sleep(1.0)
                #    rospy.loginfo(":(")
                #    continue

                #now_pose = trans.transform.translation

                #dist = math.sqrt((diff_x)**2 + (diff_y)**2)
            #rospy.loginfo("target:" + str(dist_x) + "\t" + str(dist_y) + "now:" + str(now_pose[0]) + "\t" + str(now_pose[1]))
                #if self.xy_tolerance >= dist:
            
            if (dir_x == True and dist_x <= now_pose.x) or (dir_y == True and dist_y <= now_pose.y):
                cmd_vel.linear.y = 0
                cmd_vel.linear.x = 0
                self.pub.publish(cmd_vel)
                r.finished = True
                r.pose = trans.transform
                self.server.set_succeeded(r)
                return 0
            if (dir_x == False and dist_x >= now_pose.x) or (dir_y == False and dist_y >= now_pose.y):
                cmd_vel.linear.y = 0
                cmd_vel.linear.x = 0
                self.pub.publish(cmd_vel)
                r.finished = True
                r.pose = trans.transform
                self.server.set_succeeded(r)
                return 0

                #theta = math.atan2(diff_x, diff_y)

            rate.sleep()
        cmd_vel.linear.y = 0
        cmd_vel.linear.x = 0
        self.pub.publish(cmd_vel)
        self.server.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('simple_rover_controllor')
    #xy_tolerance = rospy.get_param("~xy_tolerance", default=0.04)
    #rot_tolerance = rospy.get_param("~rot_tolerance", default=0.8)

    server = SimpleMoveServer()#xy_tolerance, rot_tolerance)
    rospy.spin()