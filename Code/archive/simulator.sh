#!/bin/bash
cd
cd Tamir/Master/Code/

dof = 6
folder = "6dof/combined"
python

sim = Simulator(dof, folder, True)





echo "Hello $i World"
ARRAY=(0 1)

for i in {1..5}
do
	# python2.7 test.py  $i "t"
	ARRAY[i]=$(python2.7 -c 'import test; print test.run()' ) # $i "t"
done
echo ${ARRAY[*]}


#tic = datetime.datetime.now()
#dofe = 6
#foldere = "6dof/combined"
#sim = Simulator(dofe, foldere, True)
#sim.run_simulation()
#toc = datetime.datetime.now()
#print('Time of Run (seconds): ' + str((toc - tic).seconds))

#    def run_simulation(self):
#        save_name = 'results_file' + datetime.datetime.now().strftime("%d_%m_%y")  # file to save the results
#        all_data = [["Date", "Time ", "Arm ", "Results "]]
#        for arm in range(0, len(self.arms)):
#
#            print "arm " + str(arm + 1) + " of " + str(len(self.arms)) + " arms"
#            if arm % 20 == 0:  # save every x iterations
#                HandleCSV().save_data(all_data, save_name)
#                all_data = []
#            data = []
#            for i in range(len(self.poses)):  # send the manipulator to the selected points
#                data.append(str(self.manipulator_move.go_to_pose_goal(self.poses[i], self.oriens[i])))
#            # inserting the data into array
#            all_data.append([datetime.datetime.now().strftime("%d/%m/%y, %H:%M"),
#                             self.arms[arm]["name"], ",".join(data)])
#            if arm == len(self.arms) - 1:
#                break
#            self.replace_model(arm)
#        # save the remaining data and close all the launch files
#        HandleCSV().save_data(all_data, save_name)
#        self.ros.stop_launch(self.arm_control)
#        self.ros.ter_command("kill -9 " + str(self.ros.checkroscorerun()))

