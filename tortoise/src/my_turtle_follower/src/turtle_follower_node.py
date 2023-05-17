#!/usr/bin/env python3

import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleFollowerNode:
    def __init__(self):
        rospy.init_node('turtle_follower')
        #Получение имени и скорости черепахи за которой следуют
        self.speed = rospy.get_param('~speed', 1.0)
        self.following_turtle_name = rospy.get_param('~following_turtle_name', 'turtle1')
        #Создание топиков для циркуляции данных
        self.following_turtle_pose_sub = rospy.Subscriber('/'+self.following_turtle_name+'/pose', Pose, self.following_turtle_pose_callback)
        self.follower_turtle_vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

        rospy.wait_for_message('/'+self.following_turtle_name+'/pose', Pose)


        self.run()

    def following_turtle_pose_callback(self, data):
        #Получение положения следующей черепахи
        self.following_turtle_pose = data
        self.is_moving = (abs(data.linear_velocity) > 0.1 or abs(data.angular_velocity) > 0.1)

    def run(self):
        rate = rospy.Rate(10)
        is_following = True

        while not rospy.is_shutdown():
            #Получение положения преследующей черепахи
            follower_turtle_pose = rospy.wait_for_message('/turtle2/pose', Pose)

            if not is_following:
                if self.following_turtle_pose is None:
                    continue
                else:
                    is_following = True
            #Ошибка по координатам
            dx = self.following_turtle_pose.x - follower_turtle_pose.x
            dy = self.following_turtle_pose.y - follower_turtle_pose.y
            #Угол поворота
            desired_yaw = math.atan2(dy, dx)
            err_yaw = desired_yaw - follower_turtle_pose.theta
            #Команда на движение преследующей черепахи и далее пропорциональный регулятор
            cmd_vel = Twist()
            cmd_vel.linear.x = self.speed
            cmd_vel.angular.z = 3.0 * err_yaw
            self.follower_turtle_vel_pub.publish(cmd_vel)

            #Расстояние между черепашками меньше 0.5 -> остановка
            if math.sqrt(dx**2 + dy**2) < 0.5:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.follower_turtle_vel_pub.publish(cmd_vel)

            rate.sleep()

if __name__ == '__main__':
    try:
        TurtleFollowerNode()
    except rospy.ROSInterruptException:
        pass
