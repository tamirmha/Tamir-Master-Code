import datetime


def run_simulation(poses, oriens, arm_name):
    data = []
    for i in range(len(poses)):  # send the manipulator to the selected points
        data.append(str(self.manipulator_move.go_to_pose_goal(poses[i], oriens[i])))
    all_data = [datetime.datetime.now().strftime("%d/%m/%y, %H:%M"), arm_name, ",".join(data)]
    return all_data
