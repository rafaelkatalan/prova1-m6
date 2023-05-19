#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import *
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

path = [Pose(x=0.0, y=0.5), Pose(x=0.5, y=0.0), Pose(x=0.0, y=0.5), Pose(x=0.5, y=0.0), Pose(x=0.0, y=1.0), Pose(x=1.0, y=0.0)]

class TurtleController(Node):
    currentPose = Pose(x = 40.0, y = 40.0)
    angleSet = False
    volta = []
    def __init__(self, path):
        self.path = path
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(
            msg_type=Twist,
            topic='/turtle1/cmd_vel',
            qos_profile=10
        )
        self.pose_subscription = self.create_subscription(
            msg_type=Pose,
            topic='/turtle1/pose',
            callback=self.pose_callback,
            qos_profile=10
        )

        self.timer_ = self.create_timer(0.1, self.move_turtle)
        self.twist_msg_ = Twist()

    def move_turtle(self):
        if self.currentPose == Pose(x = 40.0,y = 40.0):
            return
        if len(self.path)>0:
            print(f"ida{self.path}")
            nextPose = self.path[0]
        else:
            print(f"volta{self.volta}")
            nextPose = self.volta[len(self.volta)-1]
        print(f"np: {nextPose.x},{nextPose.y}")
        self.pose_subscription.callback
        print(f"cp: {self.currentPose}")
        dx = self.currentPose.x-nextPose.x
        dy = self.currentPose.y-nextPose.y
        print(f"dx={dx},dy={dy}")
        ang = atan2(dy,dx)-self.currentPose.theta
        print(f"ang: {ang}")
        if self.angleSet == False:
            if (abs(dx)<0.1 and abs(dy)<0.1):
                if len(self.path)>0:
                    print(f"ida{self.path}")
                    self.volta.append(self.path.pop(0))
                else:
                    print(f"volta{self.volta}")
                    self.volta.pop(len(self.volta)-1)
                self.angleSet = False
            if ang > 0.01 or ang < -0.01:
                self.twist_msg_.linear.x = 0.0
                self.twist_msg_.angular.z = 0.2
            else:
                self.twist_msg_.angular.z = 0.0
                self.angleSet = True
                print("Angulo ajustado!")

        if self.angleSet:
            if (abs(dx)>0.1 or abs(dy)>0.1):
                self.twist_msg_.linear.x = -0.2
            else:
                self.twist_msg_.linear.x = 0.0
                if len(self.path)>0:
                    self.volta.append(self.path.pop(0))
                else:
                    self.volta.pop(len(self.volta)-1)
                self.angleSet = False

        self.publisher_.publish(self.twist_msg_)

    def pose_callback(self, msg):
        x = msg.x
        y = msg.y
        theta = msg.theta

        self.currentPose = Pose(x=x,y=y,theta=theta)
        
def main(args=None):

    rclpy.init()
    turtle_controller = TurtleController(path)
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
