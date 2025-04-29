#!/usr/bin/env python
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from moveit_planners_ompl.srv import CustomCost, CustomCostResponse
import rospy
import moveit_commander

robot = moveit_commander.RobotCommander()

def calculate_cost(request):
    state = request.state
    state = list(state)

    # Wait for the FK service
    rospy.wait_for_service('/compute_fk')
    compute_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)

    robot_state = robot.get_current_state()
    robot_state.joint_state.position = state + [0,0]
    # FK request 
    fk_request = GetPositionFKRequest()
    fk_request.header.frame_id = "world"
    fk_request.fk_link_names = ["panda_link8"]  # Specify the end-effector link name
    fk_request.robot_state = robot_state

    # FK calculation 
    fk_response = compute_fk(fk_request)
    eef_pose = fk_response.pose_stamped[0].pose

    # print("[joint_state]",state) #You can see the state cost sampled by the OMPL
    
    cost = eef_pose.position.z # Use EE height as state cost 
    print("[joint cost]", cost)
        
    srv = CustomCostResponse()
    srv.cost = cost
    srv.type = 0
    return srv


def random_cost_server():
    rospy.init_node('cost_server')
    rospy.Service('custom_cost', CustomCost, calculate_cost)
    rospy.loginfo("Ready to compute cost.")
    rospy.spin()


if __name__ == "__main__":
    random_cost_server()