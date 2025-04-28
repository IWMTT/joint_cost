#!/usr/bin/env python

from moveit_planners_ompl.srv import CustomCost, CustomCostResponse
import rospy
import random


def calculate_cost(request):
    cost = random.random()
    state = request.state

    # print("[joint_state]",state) #You can see the state cost sampled by the OMPL

    cost = abs(state[2]) # Use absolute of 3rd joint angle as state cost 
    print("[joint cost]", cost)
        
    srv = CustomCostResponse()
    srv.cost = cost
    srv.type = 0
    return srv


def random_cost_server():
    rospy.init_node('cost_server')
    rospy.Service('custom_cost', CustomCost, calculate_cost)
    rospy.loginfo("Ready to compute random cost.")
    rospy.spin()


if __name__ == "__main__":
    random_cost_server()