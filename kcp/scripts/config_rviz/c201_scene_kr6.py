#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def main():
    rospy.init_node("c201", anonymous=True)    

    #--------------------------------

    robot = moveit_commander.RobotCommander()        
    scene = moveit_commander.PlanningSceneInterface()
    
    group_name = rospy.get_param('/c201/Planning_Group') #-------------GROUP NAME
    #group_name = "kuka6"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )
    
    planning_frame = move_group.get_planning_frame()    
    eef_link = move_group.get_end_effector_link()    
    group_names = robot.get_group_names()
    current_state = robot.get_current_state()
    dist_kuka_piso = 0.8
    #----------------------------SOPORTE KUKA

    aux_pose = geometry_msgs.msg.PoseStamped()
    aux_pose.header.frame_id = "base_link"
    aux_pose.pose.orientation.w = 1.0
    aux_pose.pose.position.x = 0.0
    aux_pose.pose.position.y = 0.0
    aux_pose.pose.position.z = -dist_kuka_piso/2
    box_name = "suport_cucaracha"
    scene.add_box(box_name, aux_pose, size=(0.3, 0.3, dist_kuka_piso))

    #----------------------------PISO

    aux_pose = geometry_msgs.msg.PoseStamped()
    aux_pose.header.frame_id = "base_link"
    aux_pose.pose.orientation.w = 1.0
    aux_pose.pose.position.x = 2.5
    aux_pose.pose.position.y = -1.0
    aux_pose.pose.position.z = 0.05 - dist_kuka_piso
    box_name = "piso"
    scene.add_box(box_name, aux_pose, size=(7.0, 4.0, 0.1))

    aux_pose = geometry_msgs.msg.PoseStamped()
    aux_pose.header.frame_id = "base_link"
    aux_pose.pose.orientation.w = 1.0
    aux_pose.pose.position.x = 3.5
    aux_pose.pose.position.y = 4.0
    aux_pose.pose.position.z = 0.05 - dist_kuka_piso
    box_name = "piso2"
    scene.add_box(box_name, aux_pose, size=(5.0, 6.0, 0.1))

    #----------------------------PAREDES

    aux_pose = geometry_msgs.msg.PoseStamped()
    aux_pose.header.frame_id = "base_link"
    aux_pose.pose.orientation.w = 1.0
    aux_pose.pose.position.x = 0.0
    aux_pose.pose.position.y = 1.0
    aux_pose.pose.position.z = 1.0 - dist_kuka_piso
    box_name = "wall_left"
    scene.add_box(box_name, aux_pose, size=(2.1, 0.1, 2.0))  
    
    aux_pose = geometry_msgs.msg.PoseStamped()
    aux_pose.header.frame_id = "base_link"
    aux_pose.pose.orientation.w = 1.0
    aux_pose.pose.position.x = -1.0
    aux_pose.pose.position.y = -1
    aux_pose.pose.position.z = 1.0 - dist_kuka_piso
    box_name = "wall_bottom"
    scene.add_box(box_name, aux_pose, size=(0.1, 4.0, 2.0))

    aux_pose = geometry_msgs.msg.PoseStamped()
    aux_pose.header.frame_id = "base_link"
    aux_pose.pose.orientation.w = 1.0
    aux_pose.pose.position.x = 0.0
    aux_pose.pose.position.y = -3.0
    aux_pose.pose.position.z = 1.0 - dist_kuka_piso
    box_name = "wall_door1"
    scene.add_box(box_name, aux_pose, size=(2.0, 0.1, 2.0))
      
    aux_pose = geometry_msgs.msg.PoseStamped()
    aux_pose.header.frame_id = "base_link"
    aux_pose.pose.orientation.w = 1.0
    aux_pose.pose.position.x = 4
    aux_pose.pose.position.y = -3.0
    aux_pose.pose.position.z = 1.0 - dist_kuka_piso
    box_name = "wall_door2"
    scene.add_box(box_name, aux_pose, size=(4.0, 0.1, 2.0))
    
    aux_pose = geometry_msgs.msg.PoseStamped()
    aux_pose.header.frame_id = "base_link"
    aux_pose.pose.orientation.w = 1.0
    aux_pose.pose.position.x = 6.0
    aux_pose.pose.position.y = 2.0
    aux_pose.pose.position.z = 1.0 - dist_kuka_piso
    box_name = "wall_front"
    scene.add_box(box_name, aux_pose, size=(0.1, 10.0, 2.0))
    
    aux_pose = geometry_msgs.msg.PoseStamped()
    aux_pose.header.frame_id = "base_link"
    aux_pose.pose.orientation.w = 1.0
    aux_pose.pose.position.x = 1.0
    aux_pose.pose.position.y = 4.0
    aux_pose.pose.position.z = 1.0 - dist_kuka_piso
    box_name = "wall_a1"
    scene.add_box(box_name, aux_pose, size=(0.1, 6.0, 2.0))
    
    aux_pose = geometry_msgs.msg.PoseStamped()
    aux_pose.header.frame_id = "base_link"
    aux_pose.pose.orientation.w = 1.0
    aux_pose.pose.position.x = 3.5
    aux_pose.pose.position.y = 7
    aux_pose.pose.position.z = 1.0 - dist_kuka_piso
    box_name = "wall_a2"
    scene.add_box(box_name, aux_pose, size=(5.0, 0.1, 2.0))

    #---------------------------- MESAS

    aux_pose = geometry_msgs.msg.PoseStamped()
    aux_pose.header.frame_id = "base_link"
    aux_pose.pose.orientation.w = 1.0
    aux_pose.pose.position.x = 3-0.25
    aux_pose.pose.position.y = 1.5
    aux_pose.pose.position.z = 0.1
    box_name = "mesa1"
    scene.add_box(box_name, aux_pose, size=(3.5, 0.8, 0.1))

    aux_pose = geometry_msgs.msg.PoseStamped()
    aux_pose.header.frame_id = "base_link"
    aux_pose.pose.orientation.w = 1.0
    aux_pose.pose.position.x = 3-0.25
    aux_pose.pose.position.y = 3.5
    aux_pose.pose.position.z = 0.1
    box_name = "mesa2"
    scene.add_box(box_name, aux_pose, size=(3.5, 0.8, 0.1))

    aux_pose = geometry_msgs.msg.PoseStamped()
    aux_pose.header.frame_id = "base_link"
    aux_pose.pose.orientation.w = 1.0
    aux_pose.pose.position.x = 2.75
    aux_pose.pose.position.y = 5.5
    aux_pose.pose.position.z = 0.1
    box_name = "mesa3"
    scene.add_box(box_name, aux_pose, size=(3.5, 0.8, 0.1))   

    #scene.setColor("mesa3", (50,50,50))

if __name__ == "__main__":
    main()