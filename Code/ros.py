#!/usr/bin/env python
# Ros Libs
import roslaunch, rospy
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist

# System Libs
import subprocess, shlex, os, time
import ntpath

from collections import OrderedDict

pathname = os.path.dirname(os.path.realpath(__file__))  # relative position- good for any pc(ubuntu)


class ros_pub_sub(object):
    '''This Class handle the massages from other PC's to the Mobile Platform's NUC'''

    def __init__(self):
        global pathname
        self.msg_rcvd = False  # does msg recived from GUI
        self.stop_run = False  # Stop run this file
        self.WebCamCaptured = False
        self.KinnectCaptured = False
        self.rosBagRun = False
        self.MappingRun = False
        self.cmd_list()
        self.pathname = pathname
        self.ros_core_start()  # Start ros runnning
        self.pub_sub_init()

    def callback(self, data, *args):
        if data._type == "geometry_msgs/Twist":
            Data = "x_ linear: " + str(data.linear.x) + "      y_ linear: " + str(
                data.linear.y) + "     z_ linear: " + str('%.2f' % data.linear.z) + '\n' + \
                   "x_ angular: " + str(data.angular.x) + "      y_ angular: " + str(
                data.angular.y) + "     z_ angular: " + str('%.2f' % data.angular.z)
        if data._type == "std_msgs/String":
            Data = data.data
            self.talker(Data)
            self.msg_handler(Data)
            self.msg_rcvd = True

    def pub_sub_init(self):
        '''Initiliaze the topics that are published and subscribed'''
        try:
            self.pubGUI = rospy.Publisher('MidLevelCommands', String, queue_size=10)
            self.pub_des_vel = rospy.Publisher('DesVel', UInt8, queue_size=10)
            rospy.Subscriber("ard_odom", Twist, self.callback)
            rospy.Subscriber("GuiCommands", String, self.callback)
        except ValueError:
            rospy.loginfo('Error occurred at pub_sub_init function')  # shows warning message
            pass

    def talker(self, msg):
        try:
            self.pubGUI.publish(msg)
        except ValueError:
            rospy.loginfo('Error occurred at talker function')  # shows warning message
            pass

    def msg_handler(self, msg):
        print self.switcher[msg]
        self.cmd = self.switcher[msg]
        # eval(self.switcher[msg])
        # self.mapping_stop()

    def a_exit(self, *args):
        self.stop_run = True
        print self.stop_run
        self.ros_core_stop()
        time.sleep(1)
        rospy.signal_shutdown("User Shutdown")

    def capture_webcam(self, *args):
        """Start webcam Capture"""
        try:
            if not self.WebCamCaptured:
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(uuid)
                path = [self.pathname + "/src/arl_description/launch/webcam.launch"]
                self.launch_Capture = roslaunch.parent.ROSLaunchParent(uuid, path)
                self.launch_Capture.start()
                self.WebCamCaptured = True
                time.sleep(0.5)
            command = "rosservice call /webcam/image_view/save"
            self.ter_command(command)
        except ValueError:
            rospy.loginfo('Error occurred at capture_webcam function')
            pass

    def capture_stop(self, *args):
        try:
            if self.WebCamCaptured:
                self.launch_Capture.shutdown()
                self.WebCamCaptured = False
        except ValueError:
            rospy.loginfo('Error occurred at capture_stop function')
            pass

    def mapping_record_start(self, *args):
        try:
            '''if not MappingRun: #Check If mapping alraedy start
                self.ARLLaunchSelect()
                MappingRun=True'''
            FileName = "Record"  # self.RecPath.get()
            command = "rosbag record -a -x /kinect2/(.*) --split --duration=1m -O " + FileName + ".bag __name:=my_bag"
            self.ter_command(command)
            self.rosBagRun = True
        except ValueError:
            rospy.loginfo('Error occurred at mapping_record_start function')  # shows warning message
            pass

    def mapping_record_stop(self, *args):
        try:
            if self.rosBagRun:
                self.ter_command("rosnode kill /my_bag")
                self.rosBagRun = False
        except ValueError:
            rospy.loginfo('Error occurred at mapping_record_stop function')  # shows warning message
            pass

    def ter_command(self, command):
        '''Write Command to the terminal'''
        try:
            command = shlex.split(command)
            ter_command_proc = subprocess.Popen(command)
        except ValueError:
            rospy.loginfo('Error occurred at ter_command function')  # shows warning message
            pass

    def ros_core_start(self, *args):
        try:
            self.roscore = subprocess.Popen('roscore')
            rospy.init_node('arl_python', anonymous=True)
            time.sleep(1)  # wait a bit to be sure the roscore is really launched
        except ValueError:
            rospy.loginfo('Error occurred at ros_core_start function')  # shows warning message
            pass

    def ros_core_stop(self, *args):
        try:
            if self.rosBagRun:
                self.ter_command("rosnode kill /my_bag")
                self.rosBagRun = False
            if self.MappingRun:
                launch_arl_describtion.shutdown()
                self.MappingRun = False
            self.roscore.terminate()
        except ValueError:
            rospy.loginfo('Error occurred at ros_core_stop function')  # shows warning message
            pass
