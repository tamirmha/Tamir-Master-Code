#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from math import pi
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonInteface(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        # initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        #  Provides information such as the robot's kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        # This provides a remote interfacefor getting, setting, and updating the robot's
        # internal understanding of the surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        # This object is an interface to a planning group (group of joints).
        # This interface can be used to plan and execute motions:
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        ## Getting Basic Information
        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link = self.move_group.get_end_effector_link()
        # # a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        # Misc variables
        self.box_name = ''
        self.cylinder_name = ''
        self.move_group.set_workspace([-10, -10, 0.2, 10, 10, 10])

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        # Planning to a Joint Goal
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, pose, orientaion):
        """send position and orientaion of the desired point
        pose - x,y,z poistion - in world frame
        orientaion - roll, pitch, yaw position - in world frame
        return true if the movement succeeded and reach at the desired accuracy
        """
        quaternion =tf.transformations.quaternion_from_euler(orientaion[0], orientaion[1], orientaion[2])
        # Planning to a Pose Goal
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]

        self.move_group.set_pose_target(pose_goal)

        # we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)  # return true if succeed false if not
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        accuracy = all_close(pose_goal, self.move_group.get_current_pose().pose, 0.01)
        return accuracy and plan

    def plan_cartesian_path(self, scale=0.5):
        #move_group = self.move_group

        # Cartesian Paths
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space,
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def execute_plan(self, plan):

        # the plan that has already been computed:
        self.move_group.execute(plan, wait=True)
        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

        ## Ensuring Collision Updates Are Receieved
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self.box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

    def add_obstacles(self, timeout=4):
        floor = {'name': 'floor', 'pose': [0, 0, -0.1], 'size': (2, 2, 0.01)}
        height = 1.0
        radius = 0.1
        rospy.sleep(0.2)
        # Adding Objects to the Planning Scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = floor['pose'][0]
        box_pose.pose.position.y = floor['pose'][1]
        box_pose.pose.position.z = floor['pose'][2]
        self.box_name = floor['name']
        self.scene.add_box(self.box_name, box_pose, size=floor['size'])
        self.scene.attach_box('world', self.box_name)
        # add plant
        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = self.robot.get_planning_frame()
        cylinder_pose.pose.orientation.w = 1.0
        cylinder_pose.pose.position.x = 2**0.5/2.0
        cylinder_pose.pose.position.y = 2**0.5/2.0
        cylinder_pose.pose.position.z = height/2.0
        self.cylinder_name= 'plant'
        self.scene.add_cylinder(self.cylinder_name,cylinder_pose, height, radius)
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


def main():
    print ""
    print "============ Press `Enter` to begin ..."

    manipulator = MoveGroupPythonInteface()
    manipulator.add_obstacles()  # add floor
    #manipulator.add_obstacles(obstacle={'name': 'plant', 'pose': [0.5, 0.5, 0.5], 'size': (0.2, 0.2, 1)})  # add plant

    #raw_input()
    print "execute a movement using a pose goal ..."
    pose_array = [[0.3, 0.1, 0.5],[-0.3, -0.1, 0.5]]
    for i in range(len(pose_array)):
        pose = pose_array[i]
        orientaion = [-0.5, 0.5, 0.3]
        print manipulator.go_to_pose_goal(pose,orientaion)

    #cartesian_plan, fraction = manipulator.plan_cartesian_path()
    #manipulator.execute_plan(cartesian_plan)


if __name__ == '__main__':
    main()
