import rospy
from geometry_msgs.msg import Twist, PoseStamped
from turtlesim.msg import Pose

class TurtleFollower:
    def __init__(self, turtle_name):
        # Подписка на топик обновления позиции первой черепашки
        rospy.Subscriber(\'/{}/pose\'.format(turtle_name), Pose, self.pose_callback)

        # Подписка на топик обновления позиции второй черепашки
        rospy.Subscriber(\'/{}/pose\'.format(self.follower_name), Pose, self.follower_pose_callback)

        # Инициализация публикатора для отправки команды на движение второй черепашки
        self.pub = rospy.Publisher(\'/{}/cmd_vel\'.format(self.follower_name), Twist, queue_size=10)

        # Инициализация позиций обеих черепашек
        self.turtle1_pose = None
        self.follower_pose = None

        # Установка имени второй черепашки
        self.follower_name = \'turtle2\'

    def pose_callback(self, data):
        # Обновление позиции первой черепашки
        self.turtle1_pose = data

    def follower_pose_callback(self, data):
        # Обновление позиции второй черепашки
        self.follower_pose = data

    def run(self):
        rate = rospy.Rate(10) # Частота публикации команд на движение второй черепашки
        while not rospy.is_shutdown():
            if self.turtle1_pose and self.follower_pose:
                # Вычисление разницы в позициях между первой и второй черепашками
                delta_x = self.turtle1_pose.x - self.follower_pose.x
                delta_y = self.turtle1_pose.y - self.follower_pose.y

                # Вычисление угла между позициями черепашек
                angle = atan2(delta_y, delta_x)

                # Создание сообщения с командой на движение второй черепашки
                msg = Twist()
                msg.linear.x = 1.0
                msg.angular.z = Kp*angle

                self.pub.publish(msg)
            rate.sleep()

if __name__ == \'__main__\':
    try:
        rospy.init_node(\'turtle_follower\')
        follower = TurtleFollower(\'turtle1\')
        follower.run()
    except rospy.ROSInterruptException:
        pass
