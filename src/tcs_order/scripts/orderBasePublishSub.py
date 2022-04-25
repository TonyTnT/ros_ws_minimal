#!/usr/bin/env python3
# endoding: utf-8

import rospy
from tcs_order.msg import OrderBase

def status_callback(msg):
    rospy.loginfo(msg)


def Show_status():
    global order_base_sub, order_base_pub
    rospy.init_node('order_base_sub')
    order_base_sub = rospy.Subscriber('/order_base/result', OrderBase, status_callback)
    rospy.spin()

if __name__ == '__main__':
    Show_status()
