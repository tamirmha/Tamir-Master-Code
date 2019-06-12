from ros import Ros, MoveGroupPythonInterface, HandleCSV
import datetime
import time


class SimulatorMoveit(object):

    def __init__(self):
        self.ros = Ros()

        # desired positions and orientaions of the EE in world frame
        self.poses = [[0.5, 0.15, 6.86], [0.5, 0.0, 6.89], [0.5, -0.15, 6.86], [0.5, -0.15, 6.45], [0.5, 0.15, 6.45]]
        self.oriens = [[1.98, -0.83, 0], [-3.14, 0, 0], [-1.98, -0.83, 0], [-0.81, 0.52, 0], [0.9, 0.02, 0]]
        # set the obstacles and initiliaze the manipulator
        # for some reason the 1st manipulator must succeed reach to point otherwise the other manipulators will failed
        self.manipulator_move = MoveGroupPythonInterface()  # for path planning and set points
        time.sleep(2)  # need time to upload
        # add floor and plant to the planning model
        self.manipulator_move.add_obstacles(height=6.75, radius=0.1, pose=[0.5, 0])
        time.sleep(2)
        self.ros.pub_sub_init(pub_name="Moveit_Pub", sub_name="Moveit_Sub")
        self.ros.talker("Moveit Ready")
        time.sleep(2)
        self.manipulator_move.go_to_pose_goal(self.poses[0], self.oriens[0])

    def handle_msg(self):
        i = 0
        msg = ""
        for i in range(100):
            msg = self.ros.data_back
            if msg != "":
                self.ros.talker("Massage Recieved")
                break
            time.sleep(0.03)
        if i == 100:  # timeout
            self.ros.talker("No Massage Recieved")
        self.ros.data_back = ""
        return msg

    def run_simulation(self):
        save_name = 'results_file' + datetime.datetime.now().strftime("%d_%m_%y")  # file to save the results
        all_data = [["Date", "Time ", "Arm ", "Results "]]
        name = self.handle_msg()
        name_prev = ""
        arm = 0
        while name != "Stop":
            time.sleep(0.1)
            name = self.handle_msg()
            data = []
            if name != name_prev and name != "Massage Recieved @ Gazebo":
                if arm % 20 == 0:  # save every x iterations
                    HandleCSV().save_data(all_data, save_name)
                    all_data = []
                for i in range(len(self.poses)):  # send the manipulator to the selected points
                    data.append(str(self.manipulator_move.go_to_pose_goal(self.poses[i], self.oriens[i])))
                # inserting the data into array
                all_data.append([datetime.datetime.now().strftime("%d/%m/%y, %H:%M"), name, ",".join(data)])
                arm = arm + 1
                name_prev = name
            else:
                print name
        HandleCSV().save_data(all_data, save_name)


tic = datetime.datetime.now()
sim = SimulatorMoveit()
sim.run_simulation()
toc = datetime.datetime.now()
print('Time of Run (seconds): ' + str((toc - tic).seconds))
