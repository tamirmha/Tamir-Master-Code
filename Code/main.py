import datetime
import time
from simulator import Simulator
import rospy
from ros import Ros

tic = datetime.datetime.now()
dofe = 6
foldere = "6dof/combined"
ros = Ros()
ros.ter_command("kill -9 " + str(ros.checkroscorerun()))
ros.ros_core_start()
rospy.init_node('arl_python', anonymous=True)

sim = Simulator(dofe, foldere, True)
arms = sim.arms

for i in range(20):
    if i != 0:
        sim = Simulator(dofe, foldere, True)
        sim.run_simulation(arms[i*20:(i+1)*20])
    else:
        sim.run_simulation(arms[0:2])
    time.sleep(3)

toc = datetime.datetime.now()
print('Time of Run (seconds): ' + str((toc - tic).seconds))
