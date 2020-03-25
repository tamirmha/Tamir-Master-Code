# import rospy
# from ros import Ros
# from rosgraph_msgs.msg import Log
#
#
# def callback(data):
#     if "Ignoring transform for child_frame_id" in data.msg:
#         Ros.ter_command("kill -9 " + str(Ros.checkroscorerun()))
#         rospy.signal_shutdown("www")
#
#
# def listener():
#     rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber("rosout", Log, callback)
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()
#     # while not rospy.is_shutdown():
#     #     rospy.rostime.wallsleep(0.5)
#
#
# if __name__ == '__main__':
#     listener()
#     print("finish")