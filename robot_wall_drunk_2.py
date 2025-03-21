#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import os

class SensorProcessor:
    def __init__(self):
        self.scan_data = None
        self.target_distance = 0.5  # 50 см

    def scan_callback(self, scan):
        self.scan_data = scan.ranges

    def get_walls(self):
        if self.scan_data is None:
            return None

        # Предполагаем, что робот находится в прямоугольной комнате
        # Разделяем данные лидара на 4 сектора (стены)
        num_sectors = 4
        sector_size = len(self.scan_data) // num_sectors
        walls = []

        for i in range(num_sectors):
            sector = self.scan_data[i * sector_size:(i + 1) * sector_size]
            min_distance = min(sector)
            walls.append(min_distance)

        return walls

    def get_closest_wall(self):
        if self.scan_data is None:
            return None, None

        # Находим минимальное расстояние до стены, которая находится не ближе 50 см
        min_distance = float('inf')
        min_index = -1

        for i, distance in enumerate(self.scan_data):
            if distance >= self.target_distance and distance < min_distance:
                min_distance = distance
                min_index = i

        if min_index == -1:
            return None, None

        # Угол до ближайшей стены
        angle_to_wall = min_index * self.scan_data.angle_increment
        return min_distance, angle_to_wall

class MotionController:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.max_speed = 0.5
        self.rotation_speed = 0.5

    def rotate_robot(self, angle):
        vel = Twist()
        vel.angular.z = self.rotation_speed if angle > 0 else -self.rotation_speed
        self.pub.publish(vel)
        rospy.sleep(abs(angle) / self.rotation_speed)
        vel.angular.z = 0
        self.pub.publish(vel)
        rospy.loginfo("Rotation complete.")

    def move_to_wall(self, distance, angle):
        vel = Twist()

        if distance > self.target_distance:
            # Параболическая зависимость скорости от расстояния
            speed = self.max_speed * (1 - (self.target_distance / distance)**2)
            vel.linear.x = speed
            vel.angular.z = angle * 0.5  # Коррекция направления
            rospy.loginfo(f"Moving towards the wall. Speed: {speed}")
        else:
            # Остановка робота
            vel.linear.x = 0
            vel.angular.z = 0
            rospy.loginfo("Stopping: target distance reached.")

        self.pub.publish(vel)

class RobotController:
    def __init__(self):
        rospy.init_node('robot_wall_drunk')
        self.sensor_processor = SensorProcessor()
        self.motion_controller = MotionController()
        self.rotation_complete = False

        # Подписка на топик /scan
        rospy.Subscriber("/scan", LaserScan, self.sensor_processor.scan_callback)

    def play_sound(self, beeps=1):
        for _ in range(beeps):
            os.system('beep -f 1000 -l 200')  # Используем beep для звукового сигнала
            rospy.sleep(0.3)

    def log_walls(self, walls):
        with open('wall_log.txt', 'a') as f:
            f.write(f"Walls distances: {walls}\n")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if not self.rotation_complete:
                # Вращение робота на 90 градусов
                self.motion_controller.rotate_robot(math.pi / 2)  # 90 градусов в радианах
                self.rotation_complete = True
            else:
                # Получаем данные о всех стенах
                walls = self.sensor_processor.get_walls()

                if walls is not None:
                    # Выводим информацию о стенах
                    print("Distances to walls:")
                    for i, wall in enumerate(walls):
                        print(f"Wall {i + 1}: {wall:.2f} meters")

                    # Логируем информацию
                    self.log_walls(walls)

                    # Запрашиваем подтверждение
                    choice = input("Choose a wall to move to (1-4) or 'q' to quit: ")
                    if choice.lower() == 'q':
                        rospy.loginfo("Exiting...")
                        break

                    try:
                        choice = int(choice)
                        if 1 <= choice <= 4:
                            # Выбираем стену
                            selected_wall_distance = walls[choice - 1]
                            if selected_wall_distance >= self.sensor_processor.target_distance:
                                # Движение к выбранной стене
                                angle = (choice - 1) * (math.pi / 2)  # Угол к выбранной стене
                                self.motion_controller.move_to_wall(selected_wall_distance, angle)
                                self.play_sound(3)  # 3 звуковых сигнала при успешном завершении
                            else:
                                rospy.logwarn("Selected wall is too close.")
                                self.play_sound(1)  # 1 звуковой сигнал при ошибке
                        else:
                            rospy.logwarn("Invalid choice.")
                            self.play_sound(1)  # 1 звуковой сигнал при ошибке
                    except ValueError:
                        rospy.logwarn("Invalid input.")
                        self.play_sound(1)  # 1 звуковой сигнал при ошибке
                else:
                    rospy.logwarn("No walls detected.")
                    self.play_sound(1)  # 1 звуковой сигнал при ошибке

            rate.sleep()

if __name__ == '__main__':
    try:
        robot_controller = RobotController()
        robot_controller.run()
    except rospy.ROSInterruptException:
        pass
