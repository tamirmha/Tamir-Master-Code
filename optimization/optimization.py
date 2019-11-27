# Variables: x1 : Joints type - array [Roll, Pitch, Pris]
#            x2 : Previous axe – array [X, Y, Z]
#            x3 : Link Length - array [0.1, 0.4, 0.7]
#            x4 : DOF – Int [3, 4, 5, 6]
# Objectives:
# 1)	Min  Degree of Redundancy     [-3:0]
# 2)	Max  manipulability\ Local Conditioning index  [0-1]
# 3)	Min  z (Mid-Range Proximity)
# 4)	Min average arrival time(s)   [0-10]
# Constrains:
# •	Sum (X3) > 1
# •	X1[0] = Roll
# •	X2[0] = Z
# •	X3[0]=0.1
# •	No more than 3 Pris in X1
# •	If (X1[i]==Roll and X2[i]==Z) than (X1[i+1]!=Roll and X2[i+1]!=Z)
# •	Arrival points : reach to one from two upper points and to the middle and bottom points
import random
import csv
import datetime
# import os


class UrdfClass(object):
    """ this class create URDF files """

    def __init__(self, links=None, joints=None, joints_axis=None, rpy=None):
        """
        :param joints: array of joints types- can be 'revolute' or 'prismatic'
        :param links: array of the links lengths in meters [must be positive float]
        the first link will be the base link,
        who is always the same(connected to the world and link1 -his joint is limited to 0)
        """
        if rpy is None:
            rpy = []
        if joints_axis is None:
            joints_axis = ['z', 'y', 'y', 'y', 'z', 'y']
        if joints is None:
            joints = ['revolute', 'prismatic', 'revolute', 'revolute', 'revolute', 'revolute']
        if links is None:
            links = [1, 1, 1, 1, 1, 1]
        self.links = links
        self.joint_data = joints
        self.axis = self.init_calc(joints, joints_axis)
        self.links_number = len(self.links)
        self.rpy = rpy
        self.weights = self.calc_weight()

    def calc_weight(self):
        """
        this function calculate the weight of the links according to accumilated weight and length of arm
        :return: weigths- the weight [kg] of each link - list of strings  (from the 2nd link)
        """
        coeffs = [8.79055, 4.2928]  # the coeffs of the linear eauation (found according UR5 and motoman)
        weights = [0]  # the wieght of each link
        acc_length = 0  # accumelated length
        acc_weight = 0  # accumelated weight
        for link in self.links[1:]:
            acc_length = acc_length + float(link)
            weights.append(round(acc_length * coeffs[0] + coeffs[1] - acc_weight, 2))
            acc_weight = acc_weight + weights[-1]
        while len(weights) < 7:
            weights.append(1)
        return [str(weight) for weight in weights]

    def urdf_data(self):
        head = '''<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro"  name="arm">
  <xacro:include filename="$(find man_gazebo)/urdf/common.gazebo.xacro" />

<link name="world" />

  <joint name="world_joint" type="fixed">
    <parent link="world" />
    <child link = "base_link" />
    <origin xyz="0 -1 0" rpy="0.0 0.0 0.0" />
  </joint>

  <xacro:include filename="$(find man_gazebo)/urdf/''' + str(self.links_number) + '''dof/transmission_''' + str(
            self.links_number) + '''dof.xacro" />
  <xacro:include filename="$(find man_gazebo)/urdf/gazebo.xacro" />

  <xacro:macro name="cylinder_inertial" params="radius length mass *origin">
    <inertial>
      <mass value="${mass}" />
      <xacro:insert_block name="origin" />
      <inertia ixx="${0.0833333 * mass * (3 * radius * radius + length * length)}" ixy="0.0" ixz="0.0"
        iyy="${0.0833333 * mass * (3 * radius * radius + length * length)}" iyz="0.0"
        izz="${0.5 * mass * radius * radius}" />
    </inertial>
  </xacro:macro>

<xacro:macro name="joint_limit" params="joint_type link_length ">
	<xacro:if value="${joint_type == 'revolute'}"  >
		<xacro:property name="joint_upper_limit" value="${pi}" />
		<xacro:property name="joint_lower_limit" value="${-pi}" />
	</xacro:if>
	<xacro:unless value="${joint_type == 'revolute'}"  >
		<xacro:property name="joint_upper_limit" value="${link_length}" />
		<xacro:property name="joint_lower_limit" value="${0}" />
	</xacro:unless>
	<limit lower="${joint_lower_limit}" upper="${joint_upper_limit}" effort="150.0" velocity="3.15"/>
</xacro:macro>


  <xacro:macro name="arm_robot" params="prefix ">'''
        inertia_parameters = '''
        <xacro:property name="base_length" value="3.25"/>
        <xacro:property name="base_radius" value="0.060" />
        <xacro:property name="link0_radius" value="0.060" /> 
            <!-- Inertia parameters -->
        <xacro:property name="base_mass" value="1.0" />
        <xacro:property name="link0_mass" value="40.7" />
        <xacro:property name="link1_mass" value="3.7" />
        <xacro:property name="link2_mass" value="''' + self.weights[1] + '''" />
        <xacro:property name="link3_mass" value="''' + self.weights[2] + '''" />
        <xacro:property name="link4_mass" value="''' + self.weights[3] + '''" />
        <xacro:property name="link5_mass" value="''' + self.weights[4] + '''" />
        <xacro:property name="link6_mass" value="''' + self.weights[5] + '''" />

        <xacro:property name="link1_radius" value="0.049" />
        <xacro:property name="link2_radius" value="0.045" />
        <xacro:property name="link3_radius" value="0.040" />
        <xacro:property name="link4_radius" value="0.035" />
        <xacro:property name="link5_radius" value="0.030" />
        <xacro:property name="link6_radius" value="0.025" /> '''
        base_link = '''

        	<!--   Base Link -->
    <link name="${prefix}base_link" >

      <collision>
			<origin xyz="0 0 ${base_length/2}" rpy="0 0 0" />
        <geometry>
			<cylinder radius="${base_radius}" length="${base_length}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertial radius="${base_radius}" length="${base_length}" mass="${base_mass}">
        <origin xyz="0.0 0.0 ${base_length/2}" rpy="0 0 0" />
      </xacro:cylinder_inertial>
    </link>

    <xacro:property name="joint0_type" value="prismatic" /> 
    <xacro:property name="joint0_axe" value="0 0 1" /> 
    <xacro:property name="link0_length" value="0.25" />
<!--  joint 0	-->
    <joint name="${prefix}joint0" type="${joint0_type}">
      <parent link="${prefix}base_link" />
      <child link = "${prefix}link0" />
      <origin xyz="0.0 ${base_radius} ${base_length + link0_radius}" rpy="${-pi/2} 0.0 0" />
      <axis xyz="${joint0_axe}" />
	  <xacro:joint_limit joint_type="${joint0_type}" link_length="${link0_length*6}"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>

<!--  link 0  -->
    <link name="${prefix}link0">
      <visual>
		<origin xyz="0 0 ${link0_radius} " rpy="0 0 0" /> 
        <geometry>
            <box size="0.1 0.1 0.2"/>
        </geometry>
      </visual>
      <collision>
		 <origin xyz="0 0 ${link0_radius}" rpy="0 0 0" /> 
        <geometry>
			 <box size="0.1 0.1 0.2"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertial radius="${link0_radius}" length="${link0_length}" mass="${link0_mass}">
        <origin xyz="0.0 0.0 ${link0_length/2}" rpy="0 0 0" />
      </xacro:cylinder_inertial>
    </link>


    '''
        data = ''

        for i in range(self.links_number):
            data = data + self.joint_create(i + 1) + self.link_create(i + 1)

        tail = '''
         <!-- fake joint - the rotation about z axe of the camera is not important -->
        <joint name="fake_joint" type="revolute">
      <parent link="${prefix}link''' + str(self.links_number) + '''" />
      <child link = "camera_link" />
      <origin xyz="0.0  0.0 ${link''' + str(self.links_number) + '''_length}" rpy="0.0 0.0 0" />
      <axis xyz="0 0 1"/>
      <xacro:joint_limit joint_type="revolute" link_length="0.1"/>
      <dynamics damping="0.0" friction="0.0"/>
        </joint>

    <!-- camera link -->
        <link name="camera_link">
          <collision>
            <geometry>
              <box size="0.01 0.01 0.01"/>
            </geometry>
            <origin rpy="0 0 0" xyz="0 0 0.005"/>
          </collision>
          <xacro:cylinder_inertial radius="0.01" length="0.01" mass="0.01">
            <origin xyz="0.0 0.0 0.005" rpy="0 0 0" />
          </xacro:cylinder_inertial>
        </link>

        <!-- ee joint -->
    <joint name="${prefix}ee_fixed_joint" type="fixed">
      <parent link="camera_link" />
      <child link = "${prefix}ee_link" />
      <origin xyz="0.0  0.0 0.01" rpy="0.0 0.0 0" />
    </joint>

<!-- ee link -->
    <link name="${prefix}ee_link">
      <collision>
        <geometry>
          <box size="0.01 0.01 0.01"/>
        </geometry>
        <origin rpy="0 0 0" xyz="0 0 0.005"/>
      </collision>
    </link>

    <xacro:arm_transmission prefix="${prefix}" />
    <xacro:arm_gazebo prefix="${prefix}" />

  </xacro:macro>
  <xacro:arm_robot prefix=""/>
</robot>  '''

        txt = head + inertia_parameters + base_link + data + tail
        return txt

    @staticmethod
    def link_create(n):
        """link- data about specific link. it buit from :
                *inertia - inertail data of the link -Mass, moment of inertia, pose in the frame
                *collision - the collision properties of a link.
                *visual - > the visual properties of the link. This element specifies the shape of the object (box, cylinder, etc.) for visualization purposes
                *velocity_decay - exponential damping of the link's velocity"""
        linkname = 'link' + str(n)
        link = ''
        if n == 1:
            link = link + '''<!--  link 1  -->
    <link name="${prefix}link1">
      <visual>
		<origin xyz="0 0 ${link1_length / 2} " rpy="0 0 0" />
        <geometry>
			<cylinder radius="${link1_radius}" length="${link1_length}"/>
        </geometry>
      </visual>
      <collision>
		 <origin xyz="0 0 ${link1_length / 2}" rpy="0 0 0" />
        <geometry>
			<cylinder radius="${link1_radius}" length="${link1_length}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertial radius="${link1_radius}" length="${link1_length}" mass="${link1_mass}">
        <origin xyz="0.0 0.0 ${link1_length / 2}" rpy="0 0 0" />
      </xacro:cylinder_inertial>
    </link>'''
        else:
            link = link + '''<!-- link ''' + str(n) + '''	-->
    <link name="${prefix}''' + linkname + '''">
      <visual>
		<origin xyz="0 0 ${''' + linkname + '''_length / 2}" rpy="0 0 0" />
        <geometry>
			<cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/>
        </geometry>
      </visual>
      <collision>
	    <origin xyz="0 0 ${''' + linkname + '''_length / 2 }" rpy="0 0 0" />
        <geometry>
		    <cylinder radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertial radius="${''' + linkname + '''_radius}" length="${''' + linkname + '''_length}" mass="${''' + linkname + '''_mass}">
        <origin xyz="0.0 0.0 ${''' + linkname + '''_length / 2 }" rpy="0 0 0" />
      </xacro:cylinder_inertial>
    </link>'''
        return link

    def calc_origin(self, n):
        # calc the origin of the link according to the previuos joint
        if self.joint_data[n - 1] == "revolute":
            if self.axis[n - 1] == '0 0 1':  # roll
                if self.rpy[n - 1] == ['0 ', '0 ', '0 ']:  # links in the same directoin
                    return "0 0 ${link" + str(n - 1) + "_length}"
                elif self.rpy[n - 1] == ['${1/2*pi} ', '0 ', '0 ']:  # links in the same directoin
                    return "0 -${link" + str(n - 1) + "_radius} ${link" + str(n - 1) + "_length}"
                elif self.rpy[n - 1] == ['0 ', '${pi/2} ', '0 ']:
                    return "0 0 ${link" + str(n) + "_radius + link" + str(n - 1) + "_length}"
                else:  # the links are perpendiculars
                    return "0 ${link" + str(n - 1) + "_radius} ${link" + str(n - 1) + "_length}"
            else:  # pitch
                if self.rpy[n - 1] == ['0 ', '0 ', '0 ']:  # around y: links are in the same directoin
                    return "0 ${link" + str(n - 1) + "_radius+link" + str(n) + "_radius} ${link" + str(
                        n - 1) + "_length}"
                elif self.rpy[n - 1] == ['0 ', '0 ', '${-pi/2} ']:  # around x: links are not in the same directoin
                    return " ${link" + str(n - 1) + "_radius+link" + str(n) + "_radius} 0 ${link" + str(
                        n - 1) + "_length}"
                else:  # round x:  the links are perpendiculars
                    return "0 0 ${link" + str(n - 1) + "_length + link" + str(n) + "_radius}"
        else:  # prismatic
            if self.rpy[n - 1] == ['0 ', '0 ', '0 ']:  # links in the same directoin
                return "0 0 ${link" + str(n - 1) + "_length}"
            else:  # the links are perpendiculars
                return "0 0 ${link" + str(n - 1) + "_length + link" + str(n) + "_radius}"

    def joint_create(self, n):
        jointname = 'joint' + str(n)

        joint = '\n<xacro:property name="' + jointname + '_type" value="' + self.joint_data[n - 1] + '"/>\n' \
                                                                                                     '<xacro:property name="' + jointname + '_axe" value="' + \
                self.axis[n - 1] + '"/>\n' \
                                   '<xacro:property name="link' + str(n) + '_length" value="' + str(
            self.links[n - 1]) + '"/>\n'

        if n == 1:
            joint = joint + '''<!--  joint 1	-->
    <joint name="${prefix}joint1" type="${joint1_type}">
      <parent link="${prefix}link0" />
      <child link="${prefix}link1" />
      <origin xyz="0.0 0.0 ${link0_length}" rpy="${pi/2} 0.0 0.0" />
      <axis xyz="${joint1_axe}"/>
	  <xacro:joint_limit joint_type="${joint1_type}" link_length="${link1_length}"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>
    '''
        else:
            orgin = self.calc_origin(n)
            rpy = self.rpy[n - 1][0] + self.rpy[n - 1][1] + self.rpy[n - 1][2]
            joint = joint + '''<!--  joint ''' + str(n) + '''	-->
    <joint name="${prefix}''' + jointname + '''" type="${''' + jointname + '''_type}">
      <parent link="${prefix}link''' + str(n - 1) + '''"/>
      <child link="${prefix}link''' + str(n) + '''" />
      <origin xyz="''' + orgin + '''" rpy="''' + rpy + '''"/>
      <axis xyz="${''' + jointname + '''_axe}"/>
	  <xacro:joint_limit joint_type="${''' + jointname + '''_type}" link_length="${link''' + str(n) + '''_length}"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>
    '''
        return joint

    @staticmethod
    def urdf_write(data, filename=str(datetime.datetime.now().minute)):
        fil = open(filename + '.urdf.xacro', 'w')
        fil.write(data)
        fil.close()

    def init_calc(self, joints, joints_axis):
        axis = []
        a = 0
        for j in joints:  # make calculations for all the joints
            axis.append(self.axis_calc(joints_axis[a]))
            a = a + 1
        return axis

    @staticmethod
    def axis_calc(axe):
        if axe == 'x':
            return '1 0 0'
        elif axe == 'y':
            return '0 1 0'
        elif axe == 'z':
            return '0 0 1'
        else:
            # warning('wrong axe input.' + axe + ' entered. returning [0 0 0] ' + str(
            #     datetime.datetime.now()))  # will print a message to the console
            return '0 0 0'


class Configs:
    def __init__(self):
        configs = self.read_data("all_configs")
        possible_configs = {"3": [], "4": [], "5": [], "6": []}
        for config in configs:
            if config[0]["dof"] == 3:
                possible_configs["3"].append(config[0])
            elif config[0]["dof"] == 4:
                possible_configs["4"].append(config[0])
            elif config[0]["dof"] == 5:
                possible_configs["5"].append(config[0])
            elif config[0]["dof"] == 6:
                possible_configs["6"].append(config[0])
        self.possible_configs = possible_configs

    def read_data(self, file_name):
        with open(file_name + ".csv", 'r') as _filehandler:
            csv_file_reader = csv.reader(_filehandler)
            data = []
            manip = []
            empty = True
            for row in csv_file_reader:
                while "" in row:
                    row.remove("")
                if len(row) > 0:
                    data.append([row[0].strip("\"").strip(" "), row[1]])
                    empty = False
                else:
                    if not empty:
                        manip.append(self.read_data_action(data))
                        data = []
                    empty = True
            # manip.append(self.read_data_action(data))  # append the last session
            return manip

    @staticmethod
    def read_data_action(data):
        manip = list(map(list, zip(*data[0:])))
        manip_array_of_dict = []
        for i in range(0, len(manip) - 1, 2):
            manip_array_of_dict.append({"joint": manip[i], "axe": manip[i + 1], "dof": len(manip[i])})
        return manip_array_of_dict


def to_urdf(interface_joints, joint_parent_axis, links, folder):
    """create the desired arm
            interface_joints- roll,pitch,yaw or prismatic
                             roll - revolute around own Z axe
                             pitch - revolute that not roll
                             pris - prismatic along
            links - length of links
            joint_parent_axis - the axe, in the parent frame, which each joint use
        """
    joints = []
    joint_axis = []
    rpy = []
    # file_name = os.environ['HOME'] + "\Tamir_Ws\src\manipulator_ros\Manipulator\man_gazebo\urdf\"
    # + str(dof) + "dof\combined\"
    file_name = ""
    rolly_number = -1
    pitchz_number = 1
    prisy_number = -1
    for i in range(len(joint_parent_axis)):
        # file_name += interface_joints[i].replace(" ", "") + "_" + joint_parent_axis[i].replace(" ", "") + "_" + \
        #              links[i].replace(".", "_")
        file_name += interface_joints[i] + "_" + joint_parent_axis[i] + "_" + str(links[i]).replace(".", "_")
        if interface_joints[i] == "roll":
            joints.append("revolute")
            joint_axis.append('z')
            if joint_parent_axis[i] == "y":
                rolly_rot = '${' + str(rolly_number) + '/2*pi} '
                rpy.append([rolly_rot, '0 ', '0 '])
                rolly_number = rolly_number * -1
            elif joint_parent_axis[i] == "x":
                rpy.append(['0 ', '${pi/2} ', '0 '])
            elif joint_parent_axis[i] == "z":
                rpy.append(['0 ', '0 ', '0 '])
        elif interface_joints[i] == "pitch":
            joints.append("revolute")
            joint_axis.append('y')
            if joint_parent_axis[i] == "y":
                rpy.append(['0 ', '0 ', '0 '])
            elif joint_parent_axis[i] == "x":
                rpy.append(['0 ', '0 ', '${-pi/2} '])
            elif joint_parent_axis[i] == "z":
                # rpy.append(['${pi/2} ', '0 ', '0 '])
                pitchz = '${' + str(pitchz_number) + '/2*pi} '
                rpy.append([pitchz, '0 ', '0 '])
                pitchz_number = pitchz_number * -1
        elif interface_joints[i] == "pris":
            joints.append("prismatic")
            joint_axis.append('z')
            if joint_parent_axis[i] == "y":
                # rpy.append(['${pi/2} ', '0 ', '0 '])
                prisy = '${' + str(prisy_number) + '/2*pi} '
                rpy.append([prisy, '0 ', '0 '])
                prisy_number = prisy_number * -1
            elif joint_parent_axis[i] == "x":
                rpy.append(['0 ', '${-pi/2} ', '0 '])
            elif joint_parent_axis[i] == "z":
                rpy.append(['0 ', '0 ', '0 '])
    arm = UrdfClass(links, joints, joint_axis, rpy)
    # arm.urdf_write(arm.urdf_data(), file_name)
    return {"arm": arm, "name": file_name, "folder": folder}


class Problem:
    def __init__(self, number_of_arms=100):
        self.possibles_configs = Configs().possible_configs
        self.number_of_arms = number_of_arms  # how many arms to create
        self.x1_options = ["roll", "pitch", "pris"]
        self.x2_options = ["x", "y", "z"]
        self.x3_options = [0.1, 0.4, 0.7]
        self.x4_options = [3, 4, 5, 6]

    def get_random_values(self):
        # set random
        x4 = random.randrange(self.x4_options[0], self.x4_options[-1]+1, 1)  # number degrees of freedom
        # the first joint is roll
        x1 = [["roll"] + list(random.choices(self.x1_options, k=x4-1)) for i in range(self.number_of_arms)]
        # the first axe is constant
        x2 = [["z"] + list(random.choices(self.x2_options, k=x4-1)) for i in range(self.number_of_arms)]
        # the first link length is 0.1
        x3 = [[0.1] + list(random.choices(self.x3_options, k=x4-1)) for i in range(self.number_of_arms)]
        return x1, x2, x3, x4

    def constrains(self, x1, x2, x3, x4):
        # filter the combinations according to constrains
        to_sim = []
        for i in range(self.number_of_arms):
            if sum(x3[i]) > 1:
                for conf in self.possibles_configs[str(x4)]:
                    if x1[i] == conf["joint"] and x2[i] == conf["axe"]:
                        to_sim.append(to_urdf(x1[i], x2[i], x3[i], x4))
                        print(conf["joint"])
                        break
        return to_sim

    def evalute(self):
        # simulator
        print(self.x4_options)
        f1 = X4                      # dof
        f2 = random.random()         # manipulability
        f3 = random.random() * 10.0  # (Mid-Range Proximity)
        f4 = random.random() * 5.0   # time
        return [f1, f2, f3, f4]


prob = Problem()
X1, X2, X3, X4 = prob.get_random_values()
to_simulation = prob.constrains(X1, X2, X3, X4)
f = prob.evalute()
