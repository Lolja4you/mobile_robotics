#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# Инициализация ROS-ноды
rospy.init_node('robot_wall_drunk')

# Целевое расстояние до стены (50 см)
target_distance = 0.5

# Максимальная скорость робота
max_speed = 0.5

# Скорость вращения робота
rotation_speed = 0.5

# Инициализация издателя для управления скоростью
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Инициализация переменной для хранения данных лидара
scan_data = None

# Флаг для завершения вращения
rotation_complete = False

# Функция для обработки данных лидара
def callback(scan):
    global scan_data, rotation_complete
    scan_data = scan.ranges

    if not rotation_complete:
        # Вращение робота на 90 градусов
        rotate_robot()
    else:
        # Движение к ближайшей стене
        move_to_wall()

# Функция для вращения робота на 90 градусов
def rotate_robot():
    global rotation_complete
    vel = Twist()
    vel.angular.z = rotation_speed
    pub.publish(vel)
    rospy.loginfo("Rotating robot...")
    rospy.sleep(1.57 / rotation_speed)  # Время для поворота на 90 градусов
    vel.angular.z = 0
    pub.publish(vel)
    rotation_complete = True
    rospy.loginfo("Rotation complete.")

# Функция для движения к ближайшей стене
def move_to_wall():
    if scan_data is None:
        rospy.logwarn("No scan data received.")
        return

    # Находим минимальное расстояние до стены
    min_distance = min(scan_data)
    min_index = scan_data.index(min_distance)

    # Угол до ближайшей стены
    angle_to_wall = min_index * scan_data.angle_increment

    # Управление скоростью робота
    vel = Twist()

    if min_distance > target_distance:
        # Параболическая зависимость скорости от расстояния
        speed = max_speed * (1 - (target_distance / min_distance)**2)
        vel.linear.x = speed
        vel.angular.z = angle_to_wall * 0.5  # Коррекция направления
        rospy.loginfo(f"Moving towards the wall. Speed: {speed}")
    else:
        # Остановка робота
        vel.linear.x = 0
        vel.angular.z = 0
        rospy.loginfo("Stopping: target distance reached.")

    pub.publish(vel)

# Подписка на топик /scan
rospy.Subscriber("/scan", LaserScan, callback)

# Основной цикл ROS
rospy.spin()