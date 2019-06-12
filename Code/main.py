import datetime
import time
from ros import Ros, MoveGroupPythonInterface, UrdfClass, HandleCSV
from simulator import Simulator


tic = datetime.datetime.now()
dofe = 6
foldere = "6dof/combined"
sim = Simulator(dofe, foldere, True)
arms = sim.arms
for i in range(20):
    if i != 0:
        sim = Simulator(dofe, foldere, True, False)
        sim.run_simulation(arms[i*20:(i+1)*20])
    else:
        sim.run_simulation(arms[0:2])
    time.sleep(3)

toc = datetime.datetime.now()
print('Time of Run (seconds): ' + str((toc - tic).seconds))
