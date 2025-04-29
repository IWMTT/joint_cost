#!/usr/bin/env python
# coding: UTF-8
import sys

import geometry_msgs.msg
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
import moveit_commander
import rospy
import tf
from geometry_msgs.msg import Quaternion, Vector3

# for graph plotting
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("test_of_customized_ompl_planner")
move_group = moveit_commander.MoveGroupCommander("panda_arm")
scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

def main():


    # remove obstacles
    scene.remove_world_object()


    # go to initial posture
    joint_goal = [-1.185, 0.0, 0.0, -2.428, 0.361, 2.513, 1.633]
    move_group.go(joint_goal,wait=True)

    # add obstacles
    pose_box1 = geometry_msgs.msg.PoseStamped()
    pose_box1.pose.position = Vector3(-0.4, 0, 1)
    pose_box1.header.frame_id = "world"
    scene.add_box("box1", pose_box1, (0.2, 2.0, 2.0))

    pose_box2 = geometry_msgs.msg.PoseStamped()
    pose_box2.pose.position = Vector3(0.3, 0, 0.15)
    pose_box2.header.frame_id = "world"
    scene.add_box("box2", pose_box2, (0.3, 0.3, 0.3))


    # set new goal
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position = Vector3(0.35,  0.6, 0.3)
    q = tf.transformations.quaternion_from_euler(0, 3.1415, 0)
    pose_goal.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    move_group.set_pose_target(pose_goal)

    # trial without the customed ompl planner
    move_group.set_planner_id("RRTstar")
    move_group.set_planning_time(10)
    histories_without_customed_path = []

    for i in range(5):
        pl_result = move_group.plan()

        if pl_result[0] is not True:
            continue

        planned_path = []
        pl_path = pl_result[1].joint_trajectory.points
        for itr in range(len(pl_path)):
            tmp_pos = list(pl_path[itr].positions)
            planned_path.append(tmp_pos)

        histories_without_customed_path.append(planned_path)
    

    # trial with the customed ompl planner
    # move_group.set_planner_id("RRTstar")
    move_group.set_planner_id("RRTstarCustom")
    move_group.set_planning_time(10)
    histories_with_customed_path = []

    for i in range(5):
        pl_result = move_group.plan()

        if pl_result[0] is not True:
            continue

        planned_path = []
        pl_path = pl_result[1].joint_trajectory.points
        for itr in range(len(pl_path)):
            tmp_pos = list(pl_path[itr].positions)
            planned_path.append(tmp_pos)

        histories_with_customed_path.append(planned_path)

    # plot
    fig1, ax1, mu1, sig1 = plot_histories(histories_without_customed_path, "Without Customized OMPL Planner")
    fig2, ax2, mu2, sig2 = plot_histories(histories_with_customed_path, "With Customized OMPL Planner")
    
    plt.show()
    input()
    moveit_commander.roscpp_shutdown()    

def plot_histories(histories_list, title = ""):
    eef_height_hist= []
    max_list = []

    for history in histories_list:
        eef_height = [calculate_eef_height(elem) for elem in history]
        normalized_hist = np.interp(np.linspace(0, len(eef_height) - 1,25), np.arange(len(eef_height)), eef_height) 
        normalized_hist = normalized_hist.tolist()
        eef_height_hist.append(normalized_hist)
        max_list.append(max(eef_height))
    fig, ax = plt.subplots(1, 1, figsize=(3.00, 3.00), dpi = 300 ) 
    ax.set_title(title, fontsize=8)
    step =  np.arange(25)
    ax.set_ylim(0, 1.5)

    mu_list = []
    sigma_list = []

    for i in range(25):
       value_at_i = [normalized_hist[i] for normalized_hist in eef_height_hist]
       mu_list.append(np.mean(np.array(value_at_i) ,axis=0))
       sigma_list.append(np.std(np.array(value_at_i), axis=0)) 
    mu_list = np.array(mu_list)
    sigma_list = np.array(sigma_list)
    
    ax.plot(step,mu_list,label="eef height (avg.)",color = "blue")
    ax.fill_between(step, mu_list+sigma_list, mu_list-sigma_list, alpha=0.2, color='blue')

    ax.set_ylabel("eef height [rad]", fontsize=8)
    ax.set_xlabel("Step [-]", fontsize=8)

    ax.legend()
    handles, labels = ax.get_legend_handles_labels()
    sigma_patch = mpatches.Patch(color='blue', alpha=0.2, label='±1σ Region')
    handles.append(sigma_patch)
    labels.append('±1σ')
    ax.legend(handles, labels)
    fig.tight_layout()
    ax.tick_params(axis='both', labelsize=8)

    return fig, ax, mu_list, sigma_list 
    

def calculate_eef_height(joint_state):
    # Wait for the FK service
    rospy.wait_for_service('/compute_fk')
    compute_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)
    robot_state = robot.get_current_state()
    robot_state.joint_state.position = joint_state + [0,0]
    # FK request 
    fk_request = GetPositionFKRequest()
    fk_request.header.frame_id = "world"
    fk_request.fk_link_names = ["panda_link8"]  # Specify the end-effector link name
    fk_request.robot_state = robot_state

    # FK calculation 
    fk_response = compute_fk(fk_request)
    eef_pose = fk_response.pose_stamped[0].pose

    return eef_pose.position.z # abs(state[1]) # Use absolute of 3rd joint angle as state cost 
    

if __name__ == "__main__":
    main()