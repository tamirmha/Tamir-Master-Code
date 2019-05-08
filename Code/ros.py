#!/usr/bin/env python

# Ros Libs
import roslaunch
import rospy
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist

# System Libs
import subprocess
import shlex
import os
import time


class Ros(object):
    """ This class is for using easly ros commands"""
    def __init__(self):
        """if needed add publish and subcribe"""
        try:
            self.pathname = os.environ['HOME'] + "/catkin_ws/src/manipulator_ros/Manipulator/"  # /Tamir_Ws
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
            time.sleep(1)
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
