# # import rospy
# # from ros import Ros
# # from rosgraph_msgs.msg import Log
# #
# #
# # def callback(data):
# #     if "Ignoring transform for child_frame_id" in data.msg:
# #         Ros.ter_command("kill -9 " + str(Ros.checkroscorerun()))
# #         rospy.signal_shutdown("www")
# #
# #
# # def listener():
# #     rospy.init_node('listener', anonymous=True)
# #     rospy.Subscriber("rosout", Log, callback)
# #     # spin() simply keeps python from exiting until this node is stopped
# #     rospy.spin()
# #     # while not rospy.is_shutdown():
# #     #     rospy.rostime.wallsleep(0.5)
# #
# #
# # if __name__ == '__main__':
# #     listener()
# #     print("finish")
# # import rospy
# #
# # param = rospy.get_param("/robot_description")
# # conf_name = param[param.index("combined/")+9:param.index(".urdf")]
# from time import time, sleep
#
#
# def clock(total):
#     ended = False
#     start_time = time()
#     while not ended:
#         sleep(1)
#         now_time = time() - start_time
#         print("\033[34m" + "\033[47m" + "Elpased  {:.0f} seconds from {} seconds".format(now_time, total) + "\033[0m")
#         if now_time >= total:
#             ended = True
#
# #
# clock(100)
