#!/usr/bin/env python3
# endoding: utf-8

import rospy
from tcs_order.msg import OrderBase

def status_callback(msg):
    print(msg)


def Show_status():
    global order_base_sub, order_base_pub
    rospy.init_node('order_base2')
    msg = OrderBase()
    msg.name = "ssssss"
    msg.state = "12"
    # print(msg)
    
    # rospy.spin()

    pub = rospy.Publisher('/order_base/result', OrderBase, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.1)  # 10Hz
    while not rospy.is_shutdown():
        rospy.loginfo("TCS ORDER RESULT PUB")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    Show_status()
