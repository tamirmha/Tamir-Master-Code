#!/usr/bin/env python
# Ros Libs
import roslaunch
import rospy
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist
# Moveit libs
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from moveit_commander.conversions import pose_to_list

# System Libs
import subprocess
import shlex
import os
import time
import sys
import copy
from math import pi
import datetime
from logging import warning


class Ros(object):
    """ This class is for using easly ros commands"""
    def __init__(self):
        """if needed add publish and subcribe"""
        try:
            self.pathname = os.environ['HOME'] + "/Tamir_Ws/src/manipulator_ros/Manipulator/"  # /Tamir_Ws
        except ValueError:
            rospy.loginfo('Error occurred at init')  # shows warning message
            pass

    def start_launch(self, launch_name, launch_path, args=[]):
        """Start launch file"""
        try:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            path = self.pathname+launch_path+"/launch/"+launch_name+".launch"
            cli_args = [path, args]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], args)]
            launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            launch.start()
            #time.sleep(0.1)
            return launch

        except ValueError:
            rospy.loginfo('Error occurred at start launch function')
            pass

    def stop_launch(self, launch):
        try:
            launch.shutdown()
            return False
        except ValueError:
            rospy.loginfo('Error occurred at launch_stop function')
            pass

    def ter_command(self, command):
        """Write Command to the terminal"""
        try:
            command = shlex.split(command)
            ter_command_proc = subprocess.Popen(command)
        except ValueError:
            rospy.loginfo('Error occurred at ter_command function')  # shows warning message
            pass

    def ros_core_start(self,):
        try:
            self.roscore = subprocess.Popen('roscore')
            rospy.init_node('arl_python', anonymous=True)
            time.sleep(1)  # wait a bit to be sure the roscore is really launched
        except ValueError:
            rospy.loginfo('Error occurred at ros_core_start function')  # shows warning message
            pass

    def ros_core_stop(self,):
        try:
            self.roscore.terminate()
        except ValueError:
            rospy.loginfo('Error occurred at ros_core_stop function')  # shows warning message
            pass

    """ pub sub functions need further checking"""
    def pub_sub_init(self, pub_name='MidLevelCommands', pub_type=String, sub_name='ard_odom', sub_type=Twist):
        '''Initiliaze the topics that are published and subscribed'''
        try:
            self.pub = rospy.Publisher(pub_name, pub_type, queue_size=10)
            rospy.Subscriber(sub_name, sub_type, self.callback)
        except ValueError:
            rospy.loginfo('Error occurred at pub_sub_init function')  # shows warning message
            pass

    def talker(self, msg):
        try:
            self.pub.publish(msg)
        except ValueError:
            rospy.loginfo('Error occurred at talker function')  # shows warning message
            pass

    def callback(self, datae):
        """ callback function when recive msg"""
        if data._type == "geometry_msgs/Twist":
            Data = "x_ linear: " + str(data.linear.x) + "      y_ linear: " + str(
                data.linear.y) + "     z_ linear: " + str('%.2f' % data.linear.z) + '\n' + \
                   "x_ angular: " + str(data.angular.x) + "      y_ angular: " + str(
                data.angular.y) + "     z_ angular: " + str('%.2f' % data.angular.z)
        if data._type == "std_msgs/String":
            Data = data.data
        return Data
        # Todo add types of msgs


class MoveGroupPythonInterface(object):
    """MoveGroupPythonIntefaceTutorial"""
    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

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

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # Getting Basic Information
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
        return self.all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, pose, orientaion):
        """send position and orientaion of the desired point
        pose - x,y,z poistion - in world frame
        orientaion - roll, pitch, yaw position - in world frame
        return true if the movement succeeded and reach at the desired accuracy
        """
        quaternion = tf.transformations.quaternion_from_euler(orientaion[0], orientaion[1], orientaion[2])
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

        accuracy = self.all_close(pose_goal, self.move_group.get_current_pose().pose, 0.01)
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

    def add_obstacles(self, height=0.8, radius=0.1, pose=[0.7, 0.7], timeout=4):
        floor = {'name': 'floor', 'pose': [0, 0, -0.02], 'size': (2, 2, 0.01)}
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
        cylinder_pose.pose.position.x = pose[0]
        cylinder_pose.pose.position.y = pose[1]
        cylinder_pose.pose.position.z = height/2.0
        self.cylinder_name = 'plant'
        self.scene.add_cylinder(self.cylinder_name,cylinder_pose, height, radius)
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        # all_equal = True
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True


class UrdfClass(object):
    """ this class create URDF files """

    def __init__(self, links=[], joints=[], joints_axis=[], rpy=[]):
        """
        :param joints: array of joints types- can be 'revolute' or 'prismatic'
        :param links: array of the links lengths in meters [must be positive float]
        the first link will be the base link, who is always the same(connected to the world and link1  - his joint is limited to 0)
        """
        if not joints:
            joints = ['revolute', 'prismatic', 'revolute', 'revolute', 'revolute', 'revolute']
        if not joints_axis:
            joints_axis = ['z', 'y', 'y', 'y', 'z', 'y']
        if not links:
            links = [1, 1, 1, 1, 1, 1]
        self.links = links
        self.joint_data = joints
        self.axis = self.init_calc(joints, joints_axis)
        self.links_number = len(self.links)
        self.rpy = rpy

    def urdf_data(self):
        head = '''<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro"  name="arm">
  <xacro:include filename="$(find man_gazebo)/urdf/common.gazebo.xacro" />
  
<link name="world" />

  <joint name="world_joint" type="fixed">
    <parent link="world" />
    <child link = "base_link" />
    <origin xyz="0 0 0" rpy="0.0 0.0 0.0" />
  </joint>

  <xacro:include filename="$(find man_gazebo)/urdf/transmission.xacro" />
  <xacro:include filename="$(find man_gazebo)/urdf/gazebo.xacro" />

  <xacro:macro name="cylinder_inertial" params="radius length mass *origin">
    <inertial>
      <mass value="${mass}" />
      <xacro:insert_block name="origin" />
      <inertia ixx="${0.0833333 * mass * (3 * radius * radius + length * length)}" ixy="0.0" ixz="0.0"
        iyy="${0.0833333 * mass * (3 * radius * radius + length * length)}" iyz="0.0"
        izz="${0.5 * mass * radius * radius}" />
    </inertial>
  </xacro:macro>

<xacro:macro name="joint_limit" params="joint_type link_length ">
	<xacro:if value="${joint_type == 'revolute'}"  >
		<xacro:property name="joint_upper_limit" value="${pi}" /> 
		<xacro:property name="joint_lower_limit" value="${-pi}" /> 
	</xacro:if>
	<xacro:unless value="${joint_type == 'revolute'}"  >
		<xacro:property name="joint_upper_limit" value="${2*link_length}" /> 
		<xacro:property name="joint_lower_limit" value="${0}" />  
	</xacro:unless>
	<limit lower="${joint_lower_limit}" upper="${joint_upper_limit}" effort="150.0" velocity="3.15"/>
</xacro:macro>


  <xacro:macro name="arm_robot" params="prefix ">'''
        inertia_parameters = '''
        <xacro:property name="base_length" value="0.25"/>
            <!-- Inertia parameters -->
        <xacro:property name="base_mass" value="4.0" /> 
        <xacro:property name="link1_mass" value="3.7" />
        <xacro:property name="link2_mass" value="8.393" />
        <xacro:property name="link3_mass" value="2.275" />
        <xacro:property name="link4_mass" value="1.219" />
        <xacro:property name="link5_mass" value="1.219" />
        <xacro:property name="link6_mass" value="0.1879" />  

	    <xacro:property name="base_radius" value="0.060" /> 
        <xacro:property name="link1_radius" value="0.060" /> 
        <xacro:property name="link2_radius" value="0.060" />   
        <xacro:property name="link3_radius" value="0.060" />  
        <xacro:property name="link4_radius" value="0.040" />      
        <xacro:property name="link5_radius" value="0.030" />   
        <xacro:property name="link6_radius" value="0.025" /> '''
        base_link = '''

        	<!--   Base Link -->
    <link name="${prefix}base_link" >
      <visual>
		<origin xyz="0 0 ${base_length/2}" rpy="0 0 0" /> 
        <geometry>
 			<cylinder radius="${base_radius}" length="${base_length}"/> 
        </geometry>
      </visual>
      <collision>
			<origin xyz="0 0 ${base_length/2}" rpy="0 0 0" /> 
        <geometry>
			<cylinder radius="${base_radius}" length="${base_length}"/>	
        </geometry>
      </collision>
      <xacro:cylinder_inertial radius="${base_radius}" length="${base_length}" mass="${base_mass}">
        <origin xyz="0.0 0.0 ${base_length/2}" rpy="0 0 0" />
      </xacro:cylinder_inertial>
    </link>
    '''
        data = ''

        for i in range(self.links_number):
            data = data + self.joint_create(i + 1) + self.link_create(i + 1)

        tail = '''    
    <joint name="${prefix}ee_fixed_joint" type="fixed">
      <parent link="${prefix}link''' + str(self.links_number) + '''" />
      <child link = "${prefix}ee_link" />
      <origin xyz="0.0  0.0 ${link6_length}" rpy="0.0 0.0 0" />
    </joint>

<!-- ee link -->
    <link name="${prefix}ee_link">
      <collision>
        <geometry>
          <box size="0.01 0.01 0.01"/>
        </geometry>
        <origin rpy="0 0 0" xyz="0 0 0.005"/>
      </collision>
    </link>

    <xacro:arm_transmission prefix="${prefix}" />
    <xacro:arm_gazebo prefix="${prefix}" />

  </xacro:macro>
  <xacro:arm_robot prefix=""/>
</robot>  '''

        txt = head + inertia_parameters + base_link + data + tail
        return txt

    def link_create(self, n):
        '''link- data about specific link. it buit from :
                *inertia - inertail data of the link -Mass, moment of inertia, pose in the frame
                *collision - the collision properties of a link.
                *visual - > the visual properties of the link. This element specifies the shape of the object (box, cylinder, etc.) for visualization purposes
                *velocity_decay - exponential damping of the link's velocity'''
        linkname = 'link' + str(n)
        link = ''
        if n == 1:
            link = link + '''<!--  link 1  -->
    <link name="${prefix}link1">
      <visual>
		<origin xyz="0 0 ${link1_length / 2} " rpy="0 0 0" /> 
        <geometry>
			<cylinder radius="${link1_radius}" length="${link1_length}"/>	 
        </geometry>
      </visual>
      <collision>
		 <origin xyz="0 0 ${link1_length / 2}" rpy="0 0 0" /> 
        <geometry>
			<cylinder radius="${link1_radius}" length="${link1_length}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertial radius="${link1_radius}" length="${link1_length}" mass="${link1_mass}">
        <origin xyz="0.0 0.0 ${link1_length / 2}" rpy="0 0 0" />
      </xacro:cylinder_inertial>
    </link>'''
        else:
            link = link + '''<!-- link ''' + str(n) + '''	-->
    <link name="${prefix}''' + linkname + '''">
      <visual>
		<origin xyz="0 0 ${''' + linkname + '''_length / 2}" rpy="0 0 0" />
        <geometry>
			<cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/> 
        </geometry>
      </visual>
      <collision>
	    <origin xyz="0 0 ${''' + linkname + '''_length / 2 }" rpy="0 0 0" />
        <geometry>
		<cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertial radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}" mass="${''' + linkname + '''_mass}">
        <origin xyz="0.0 0.0 ${''' + linkname + '''_length / 2 }" rpy="0 0 0" />
      </xacro:cylinder_inertial>
    </link>'''
        return link

    def calc_origin(self, n):
        # if n == 2:

        if self.joint_data[n-1] == "revolute":
            if self.axis[n - 1] == '0 0 1':  # roll
                if self.rpy[n-1] == ['0 ', '0 ', '0 ']:  # links in the same directoin
                    return "0 0 ${link" + str(n-1) + "_length}"
                else:  # the links are perpendiculars
                    return "0 ${link" + str(n-1) + "_radius} ${link" + str(n-1) + "_length}"
            else:  # pitch
                if self.rpy[n-1] == ['0 ', '0 ', '0 '] :  # links in the same directoin
                    return "0 ${link" + str(n-1) + "_radius+link"+ str(n) + "_radius} ${link" + str(n-1) + "_length}"
                elif self.rpy[n-1] == ['0 ', '0 ', '${-pi/2} ']:  # links in the same directoin
                    return " ${link" + str(n-1) + "_radius+link"+ str(n) + "_radius} 0 ${link" + str(n-1) + "_length}"
                else:  # the links are perpendiculars
                    return "0 0 ${link" + str(n-1) + "_length + link" + str(n) + "_radius}"
        else:  # prismatic
            if self.rpy[n-1] == ['0 ', '0 ', '0 ']:  # links in the same directoin
                return "0 0 ${link" + str(n - 1) + "_length}"
            else:  # the links are perpendiculars
                return "0 0 ${link" + str(n-1) + "_length + link" + str(n) + "_radius}"

    def joint_create(self, n):
        jointname = 'joint' + str(n)

        joint = '\n<xacro:property name="' + jointname + '_type" value="' + self.joint_data[n - 1] + '"/>\n' \
                                                                                                     '<xacro:property name="' + jointname + '_axe" value="' + \
                self.axis[n - 1] + '"/>\n' \
                                   '<xacro:property name="link' + str(n) + '_length" value="' + str(
            self.links[n - 1]) + '"/>\n'

        if n == 1:
            joint = joint + '''<!--  joint 1	-->
    <joint name="${prefix}joint1" type="${joint1_type}">
      <parent link="${prefix}base_link" />
      <child link="${prefix}link1" />
      <origin xyz="0.0 0.0 ${base_length}" rpy="0.0 0.0 0.0" />
      <axis xyz="${joint1_axe}"/>
	  <xacro:joint_limit joint_type="${joint1_type}" link_length="${link1_length}"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>
    '''
        else:
            orgin = self.calc_origin(n)
            rpy = self.rpy[n - 1][0] + self.rpy[n - 1][1] + self.rpy[n - 1][2]
            joint = joint + '''<!--  joint ''' + str(n) + '''	-->
    <joint name="${prefix}''' + jointname + '''" type="${''' + jointname + '''_type}">
      <parent link="${prefix}link''' + str(n - 1) + '''"/>
      <child link="${prefix}link''' + str(n) + '''" />
      <origin xyz="''' + orgin + '''" rpy="''' + rpy + '''"/>
      <axis xyz="${''' + jointname + '''_axe}"/>
	  <xacro:joint_limit joint_type="${''' + jointname + '''_type}" link_length="${link''' + str(n) + '''_length}"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>
    '''
        return joint

    def urdf_write(self, data, filename=str(datetime.datetime.now().minute)):
        file = open(filename + '.urdf.xacro', 'w')
        file.write(data)
        file.close()

    def init_calc(self, joints, joints_axis):
        axis = []
        a = 0
        for j in joints:  # make calculations for all the joints
            axis.append(self.axis_calc(joints_axis[a]))
            a = a + 1
        return axis

    def axis_calc(self, axe):
        if axe == 'x':
            return '1 0 0'
        elif axe == 'y':
            return '0 1 0'
        elif axe == 'z':
            return '0 0 1'
        else:
            warning('wrong axe input.' + axe + ' entered. returning [0 0 0] ' + str(
                datetime.datetime.now()))  # will print a message to the console
            return '0 0 0'


def main_move_group():
    print ""
    print "============ Press `Enter` to begin ..."

    manipulator = MoveGroupPythonInteface()
    time.sleep(2)
    manipulator.add_obstacles()  # add floor
    #manipulator.add_obstacles(obstacle={'name': 'plant', 'pose': [0.5, 0.5, 0.5], 'size': (0.2, 0.2, 1)})  # add plant

    #raw_input()
    # print "execute a movement using a pose goal ..."
    # pose_array = [[0.1, 0.2, 0.6], [0.3, 0.1, 0.1+0.3]]
    # for i in range(len(pose_array)):
    #     pose = pose_array[i]
    #     orientaion = [0, -3.14, 00]
    #     print manipulator.go_to_pose_goal(pose,orientaion)
    #
    # #cartesian_plan, fraction = manipulator.plan_cartesian_path()
    #manipulator.execute_plan(cartesian_plan)


if __name__ == '__main__':
    ros = Ros()
    launchs_path = "man_gazebo"
    #main = ros.start_launch("main", launchs_path)
    k = 120
    z = 30
    a = True
    f = False
    while k > -5:
        time.sleep(1)
        k = k-1
        z = z-1

        print k
        # print k
        # if z < 0:
        #     z = 30
        #     replace = ros.start_launch("replace_model",launchs_path)
        #     if f:
        #         ros.stop_launch(arm_control)
        #     arm_control = ros.start_launch("replace_model2", launchs_path)
        #     f = True
        # if k < 0 and a:
        #     ros.stop_launch(replace)
        #     ros.stop_launch(arm_control)
        #     a = ros.stop_launch(main)
