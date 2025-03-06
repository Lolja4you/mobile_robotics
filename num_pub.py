import rospy
from std_msgs.msg import String
from random import randint

rospy.init_node("num_pub")

pub = rospy.Publisher("/numbers", String, queue_size=10)

s = String()

rate = rospy.Rate(5)
while not rospy.is_shutdown():
    s.data = f"{randint(0, 100)}"
    pub.publish(s) #в топик публикуем
    rate.sleep()
