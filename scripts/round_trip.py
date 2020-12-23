#!/usr/bin/env python

import rospy
import actionlib
from simple_rover_move.msg import MoveAction, MoveGoal

client = actionlib.SimpleActionClient('move_rover', MoveAction)

dist = rospy.get_param("~dist", default=1.0)
vel = rospy.get_param("~vel", default=0.11)
axis = rospy.get_param("~axis", default='x') #x/y/xy
times = rospy.get_param("~times", default=10)

def main():
    rospy.loginfo("vel:" + str(vel) + "\tdist:" + str(dist))
    for i in range(times):
        for j in range(2):
            rospy.loginfo("phase:" + str(i) + "-" + str(j))
            goal = MoveGoal()
            goal.dist = dist
            if axis == 'x':
                goal.vel.linear.x = vel * (1 if j == 0 else -1)
            elif axis == 'y':
                goal.vel.linear.y = vel * (1 if j == 0 else -1)
            elif axis == "xy":
                goal.vel.linear.x = vel * (1 if j == 0 else -1)/ math.sqrt(2)
                goal.vel.linear.y = vel * (1 if j == 0 else -1)/ math.sqrt(2)

            client.wait_for_server()
            client.send_goal(goal)
            client.wait_for_result()

            rospy.loginfo(client.get_result())

if __name__ == '__main__':
    rospy.init_node('rover_round_trip')
    main()
    rospy.spin()
    