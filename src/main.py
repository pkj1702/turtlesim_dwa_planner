#!/usr/bin/env python
import math
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import dynamic_window_approach as dwa
class Config:
    def __init__(self):
        self.max_speed = 3.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 90.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 5.0  # [m/ss]
        self.max_delta_yaw_rate = 90.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.3  # [m/s]
        self.yaw_rate_resolution = 0.5 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 1.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
    
        self.robot_radius = 1.0  # [m] for collision check
        self.ob = np.empty((1,2))
        # waypoints
        self.waypoints = np.array([[8.58, 2.5],
                              [2.5, 8.58],
                              [2.5, 2.5],
                              [8.58, 8.58]
                              ])

class Turtle:
    def __init__(self):
        rospy.init_node('main', anonymous=True)
        self.ob_sub = rospy.Subscriber("turtle1/pose", Pose, self.ob_callback)
        self.pos_sub = rospy.Subscriber("turtle2/pose", Pose, self.pose_callback)
        self.vel_pub = rospy.Publisher("turtle2/cmd_vel", Twist, queue_size=1)

        self.config= Config()
        self.x=0 ## need to fix
        self.turtle2_pose = Pose()
        self.goal_idx = 0

        #init pose variables
        self.turtle2_x = 0
        self.turtle2_y = 0
        self.turtle2_theta = 0
        self.turtle2_v = 0
        self.turtle2_omega = 0

    def ob_callback(self, msg):
        self.config.ob = np.array([[msg.x, msg.y]]) # 

    def pose_callback(self, msg):
        self.turtle2_pose = msg
        self.turtle2_x = self.turtle2_pose.x
        self.turtle2_y = self.turtle2_pose.y
        self.turtle2_theta = self.turtle2_pose.theta
        self.turtle2_v = self.turtle2_pose.linear_velocity
        self.turtle2_omega = self.turtle2_pose.angular_velocity

    def main(self):
        ob = self.config.ob
        if self.goal_idx == len(self.config.waypoints):
            self.goal_idx=0
        goal = self.config.waypoints[self.goal_idx]
        
        # initial state
        self.x = np.array([self.turtle2_x, self.turtle2_y, self.turtle2_theta, 
                           self.turtle2_v, self.turtle2_omega])

        # input [forward speed, yaw_rate]
        u, _ = dwa.dwa_control(self.x, self.config, goal, ob)
        msg = Twist()
        msg.linear.x = u[0]
        msg.angular.z = u[1]
        self.vel_pub.publish(msg)
        print(u)
        # check reaching goal
        dist_to_goal = math.hypot(self.x[0] - goal[0], self.x[1] - goal[1])
        if dist_to_goal <= self.config.robot_radius:
            print("------------[Goal]-------------")
            self.goal_idx +=1

if __name__=='__main__':
    try:
        turtle=Turtle() 
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():    
            turtle.main()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass