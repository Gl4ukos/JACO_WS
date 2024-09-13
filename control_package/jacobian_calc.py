#!/usr/bin/env python2
import rospy 
import moveit_commander
import numpy as np
from math import pi
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Float64

def calc_it():
    rospy.init_node('jacobian_calc', anonymous=True)
    moveit_commander.roscpp_initialize([])

    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    pub = rospy.Publisher("/jacobian_det", Float64, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        joint_values = move_group.get_current_joint_values()
        jacobian = move_group.get_jacobian_matrix(joint_values)
        jacobian_np = np.array(jacobian).reshape(6,len(joint_values))

        try:
            jacobian_det = np.linalg.det(jacobian_np)
        except np.linalg.LinAlgError:
            jacobian_det = float('nan')

        det_msg = Float64()
        det_msg.data = jacobian_det
        pub.publish(det_msg)

        rospy.loginfo("Jacobian Determinant: %f", jacobian_det)

        rate.sleep()

calc_it()
