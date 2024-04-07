#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    # 初始化ROS节点
    rospy.init_node('talker', anonymous=True)

    # 创建一个新的ROS发布者，发布的主题为'chatter'，消息类型为String
    pub = rospy.Publisher('chatter', String, queue_size=10)

    # 设置发布频率为10Hz
    rate = rospy.Rate(10)

    # 循环发布消息，直到节点被关闭
    while not rospy.is_shutdown():
        # 构造要发布的消息
        hello_str = "hello world %s" % rospy.get_time()

        # 发布消息
        rospy.loginfo(hello_str)
        pub.publish(hello_str)

        # 等待足够的时间以满足指定的发布频率
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
