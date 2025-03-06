#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_robot():
    # Инициализация ROS-ноды
    rospy.init_node('move_robot_node', anonymous=True)

    # Создание издателя для топика /cmd_vel
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Создание сообщения Twist
    vel_msg = Twist()

    # Установка линейной скорости (минимальная скорость)
    vel_msg.linear.x = 0.05  # 5 см/с (минимальная скорость)
    vel_msg.angular.z = 0.0  # Угловая скорость равна 0

    # Расстояние, которое нужно пройти (2 метра 30 сантиметров)
    distance = 2.3  # метры

    # Время, необходимое для прохождения расстояния
    time_to_move = distance / vel_msg.linear.x

    # Начало движения
    rospy.loginfo("Moving the robot...")
    start_time = rospy.Time.now().to_sec()

    while (rospy.Time.now().to_sec() - start_time) < time_to_move:
        pub.publish(vel_msg)
        rospy.sleep(0.1)  # Небольшая задержка для плавности

    # Остановка робота
    vel_msg.linear.x = 0.0
    pub.publish(vel_msg)
    rospy.loginfo("Robot has stopped.")

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass