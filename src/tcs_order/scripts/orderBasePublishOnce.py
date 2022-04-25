#!/usr/bin/env python3
# endoding: utf-8

import rospy
from tcs_order.msg import OrderBase
from std_msgs.msg import Header

def status_callback(msg):
    print(msg)


def Show_status():
    global order_base_sub, order_base_pub
    rospy.init_node('order_base2')
    msg = OrderBase()
    msg.header = Header()
    msg.name = "ssssss"
    msg.state = "12"
    # print(msg)
    
    # rospy.spin()

    pub = rospy.Publisher('/order_base/result', OrderBase, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5)  # 10Hz
    while not rospy.is_shutdown():
        rospy.loginfo("TCS ORDER RESULT PUB")
        pub.publish(msg)
        rate.sleep()
        break
    rospy.loginfo("TCS ORDER DONE")

if __name__ == '__main__':
    Show_status()
