# import rospy
# from rosgraph_msgs.msg import Log
# import os
# from Other import save_json
#
#
# def callback(data):
#     if "Ignoring transform for child_frame_id" in data.msg:
#         # Get the problematic configuration name
#         param = rospy.get_param("/robot_description")
#         conf_name = param[param.index("combined/") + 9:param.index(".urdf")]
#         save_json("no_good_confs", conf_name)
#         cmd = "kill -9 $(ps aux | grep [r]os | grep -v grep | grep -v arya | awk '{print $2}')"
#         os.system(cmd)
#         with open("finish.txt", "w+") as f:
#             f.write("finish")
#             f.close()
#
#
# def listener():
#     rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber("rosout", Log, callback)
#     rospy.spin()


if __name__ == '__main__':
    listener()
    print("ssda")