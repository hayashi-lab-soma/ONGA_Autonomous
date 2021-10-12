#!/usr/bin/env python                                                                     

import rospy
import dynamic_reconfigure.client

if __name__ == '__main__':
    rospy.init_node("dynamic_reconfig")
    client = dynamic_reconfigure.client.Client("/move_base/TrajectoryPlannerROS/")
    while not rospy.is_shutdown():
        client.update_configuration({"acc_lim_x": 0.2})
        rospy.sleep(1)
        # client.update_configuration({"TrajectoryPlannerROS/acc_lim_theta": 0.2})
        # rospy.sleep(1)
        # client.update_configuration({"DWAPlannerROS/acc_lim_x": 0.2})
        # rospy.sleep(1)
        # client.update_configuration({"DWAPlannerROS/acc_lim_th": 0.2})
        # rospy.sleep(1)

    rospy.spin()