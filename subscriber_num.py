import rospy
from std_msgs.msg import String

class GreetingWorker(object):

    def __init__(self):
        rospy.loginfo('Starting Node #1 subscriber_num')
        # Typical - bruh - Подписка на топик /name
        # self.sub_name = rospy.Subscriber('/name', String, self.cbName)
        # Subscriber 
            # 1 arg = url int ros topic
            # 2 arg = type data 
            # 3 arg = method for recive and как это аh да для analytic 
        # self.name = ""
        # Подписка на топик /numbers
        self.sub_numbers = rospy.Subscriber('/numbers', String, self.cbNumbers)
        self.number = ""
        self.n = 0
    # OBRABOTKA DATY
    # def cbName(self, income_msg):
        # self.name = income_msg.data
    def cbNumbers(self, income_msg):
        num = int(income_msg.data)
        self.number += income_msg.data + " "  
        self.n += 1
        rospy.loginfo(f"Received number: {income_msg.data}")
    # DA
    def run(self):
        while not rospy.is_shutdown():
            if self.n < 5:
                rospy.sleep(1)  
            else:
                numbers_list = list(map(int, self.number.strip().split()))
                if not numbers_list:
                    rospy.logwarn("No valid numbers received.")
                    rospy.signal_shutdown("No valid numbers")
                    break
                rospy.loginfo(f"Received numbers: {numbers_list}")
                average = sum(numbers_list) / len(numbers_list)
                rospy.loginfo(f"Average: {average}")
                rospy.signal_shutdown("Task completed")
                break
            
rospy.init_node('greeting_indep_node')
greeter = GreetingWorker()
greeter.run()