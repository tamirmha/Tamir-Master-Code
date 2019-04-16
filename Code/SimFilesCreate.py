''' i Will create many URDF files as the number of manipulators that i want to simulate
links lengths- check from [0.1-1]meter in intervals of 0.3 meter
# of DOF- check between 3-6 DOF
type of DOF - check 2 types - prismatic and revolute
order- for each # and type of DOF i will check all the possible options
'''
import datetime
import itertools
import os
from logging import warning
import numpy as np

# default data
links = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
joints_type = ['revolute', 'prismatic', 'revolute', 'revolute', 'revolute', 'revolute']
joints_axis = ['z', 'y', 'y', 'y', 'z', 'y']
#


class UrdfClass(object):
    """ this class create URDF files """

    def __init__(self, links=[], joints=[], joints_axis=[], rpy=[]):
        """
        :param joints: array of joints types- can be 'revolute' or 'prismatic'
        :param links: array of the links lengths in meters [must be positive float]
        the first link will be the base link, who is always the same(connected to the world and link1  - his joint is limited to 0)
        """
        if not joints:
            joints = ['revolute', 'prismatic', 'revolute', 'revolute', 'revolute', 'revolute']
        if not joints_axis:
            joints_axis = ['z', 'y', 'y', 'y', 'z', 'y']
        if not links:
            links = [1, 1, 1, 1, 1, 1]
        self.links = links
        self.joint_data = joints
        self.axis = self.init_calc(joints, joints_axis)
        self.links_number = len(self.links)
        self.rpy=rpy

    def urdf_data(self):
        head = '''<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro">

  <xacro:include filename="$(find ur_description)/urdf/ur.transmission.xacro" />
  <xacro:include filename="$(find ur_description)/urdf/ur.gazebo.xacro" />

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
		<xacro:property name="joint_upper_limit" value="${2*link_length}" /> 
		<xacro:property name="joint_lower_limit" value="${0}" />  
	</xacro:unless>
	<limit lower="${joint_lower_limit}" upper="${joint_upper_limit}" effort="150.0" velocity="3.15"/>
</xacro:macro>


  <xacro:macro name="ur5_robot" params="prefix joint_limited">'''
        inertia_parameters = '''
        <xacro:property name="base_length" value="0.05"/>
            <!-- Inertia parameters -->
        <xacro:property name="base_mass" value="4.0" /> 
        <xacro:property name="link1_mass" value="3.7" />
        <xacro:property name="link2_mass" value="8.393" />
        <xacro:property name="link3_mass" value="2.275" />
        <xacro:property name="link4_mass" value="1.219" />
        <xacro:property name="link5_mass" value="1.219" />
        <xacro:property name="link6_mass" value="0.1879" />  

	    <xacro:property name="base_radius" value="0.060" /> 
        <xacro:property name="link1_radius" value="0.060" /> 
        <xacro:property name="link2_radius" value="0.060" />   
        <xacro:property name="link3_radius" value="0.060" />  
        <xacro:property name="link4_radius" value="0.040" />      
        <xacro:property name="link5_radius" value="0.030" />   
        <xacro:property name="link6_radius" value="0.025" /> '''
        base_link = '''

        	<!--   Base Link -->
    <link name="${prefix}base_link" >
      <visual>
		<origin xyz="0 0 -0.075" rpy="0 0 0" /> 
        <geometry>
 			<cylinder radius="${base_radius}" length="${base_length}"/> 
        </geometry>
      </visual>
      <collision>
			<origin xyz="0 0 -0.0" rpy="0 0 0" /> 
        <geometry>
			<cylinder radius="${base_radius}" length="${base_length}"/>	
        </geometry>
      </collision>
      <xacro:cylinder_inertial radius="${base_radius}" length="${base_length}" mass="${base_mass}">
        <origin xyz="0.0 0.0 -0.075" rpy="0 0 0" />
      </xacro:cylinder_inertial>
    </link>
    '''
        data = ''

        for i in range(self.links_number):
            data = data + self.joint_create(i + 1) + self.link_create(i + 1)

        tail = '''    
    <joint name="${prefix}ee_fixed_joint" type="fixed">
      <parent link="${prefix}link''' + str(self.links_number) + '''" />
      <child link = "${prefix}ee_link" />
      <origin xyz="0.0 ${link6_length} 0.0" rpy="0.0 0.0 ${pi/2.0}" />
    </joint>

<!-- ee link -->
    <link name="${prefix}ee_link">
      <collision>
        <geometry>
          <box size="0.01 0.01 0.01"/>
        </geometry>
        <origin rpy="0 0 0" xyz="-0.01 0 0"/>
      </collision>
    </link>

    <xacro:ur_arm_transmission prefix="${prefix}" />
    <xacro:ur_arm_gazebo prefix="${prefix}" />



  </xacro:macro>
</robot>  '''

        txt = head + inertia_parameters + base_link + data + tail
        return txt

    def link_create(self, n):
        '''link- data about specific link. it buit from :
                *inertia - inertail data of the link -Mass, moment of inertia, pose in the frame
                *collision - the collision properties of a link.
                *visual - > the visual properties of the link. This element specifies the shape of the object (box, cylinder, etc.) for visualization purposes
                *velocity_decay - exponential damping of the link's velocity'''
        linkname = 'link' + str(n)
        link = ''
        if n == 1:
            link = link + '''<!--  link 1  -->
    <link name="${prefix}link1">
      <visual>
		<origin xyz="0 0 ${base_length/2} " rpy="0 0 0" /> 
        <geometry>
			<cylinder radius="${link1_radius}" length="${link1_length}"/>	 
        </geometry>
      </visual>
      <collision>
		 <origin xyz="0 0 ${base_length/2}" rpy="0 0 0" /> 
        <geometry>
			<cylinder radius="${link1_radius}" length="${link1_length}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertial radius="${link1_radius}" length="${link1_length}" mass="${link1_mass}">
        <origin xyz="0.0 0.0 ${base_length/2}" rpy="0 0 0" />
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
        if n == 2:
            return '0.0 ${link2_radius + link1_radius} ${link1_length /2 +0.025}'
        elif n == 3:
            return '0.0 ${link3_radius+link2_radius} ${link2_length}'
        elif n == 4:
            return '0 -${(link4_radius + link3_radius } ${link3_length}'
        elif n == 5:
            return '0 0 ${link4_length+link5_radius}'
        elif n == 6:
            return '0 0 ${link5_length }'

    # def calc_rpy(self, n):
    #     if n == 2:
    #         return '0 0 0'
    #     elif n == 3:
    #         return '0 0 0'
    #     elif n == 4:
    #         return '0 0 0'
    #     elif n == 5:
    #         return '${-pi/2} 0 0'
    #     elif n == 6:
    #         return '0 0 0'

    def joint_create(self, n):
        jointname = 'joint' + str(n)
        orgin = self.calc_origin(n)
        rpy = self.rpy[n-1]#calc_rpy(n)
        joint = '\n<xacro:property name="' + jointname + '_type" value="' + self.joint_data[n - 1] + '"/>\n' \
                                                                                                     '<xacro:property name="' + jointname + '_axe" value="' + \
                self.axis[n - 1] + '"/>\n' \
                                   '<xacro:property name="link' + str(n) + '_length" value="' + str(
            self.links[n - 1]) + '"/>\n'

        if n == 1:
            joint = joint + '''<!--  joint 1	-->
    <joint name="${prefix}joint1" type="${joint1_type}">
      <parent link="${prefix}base_link" />
      <child link="${prefix}link1" />
      <origin xyz="0.0 0.0 ${(link1_length-0.15)/2}" rpy="0.0 0.0 0.0" />
      <axis xyz="${joint1_axe}"/>
	  <xacro:joint_limit joint_type="${joint1_type}" link_length="${link1_length}"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>
    '''
        else:
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

    def urdf_write(self, data, filename=str(datetime.datetime.now().minute)):
        file = open(filename + '.urdf.xacro', 'w')
        file.write(data)
        file.close()

    def init_calc(self, joints, joints_axis):
        axis = []
        a = 0
        for j in joints:  # make calculations for all the joints
            # self.type.append('\''+j+'\'')
            axis.append(self.axis_calc(joints_axis[a]))
            a = a + 1
        return axis

    def axis_calc(self, axe):
        if axe == 'x':  # NO OPTION
            return '1 0 0'
        elif axe == 'y':
            return '0 1 0'
        elif axe == 'z':
            return '0 0 1'
        else:
            warning('wrong axe input.' + axe + ' entered. returning [0 0 0] ' + str(
                datetime.datetime.now()))  # will print a message to the console
            return '0 0 0'


class ToSimulate(object):
    def __init__(self,number):
        link_min = 0.1; link_interval = 0.3;link_max = 1.1
        lengths_2_check = np.arange(link_min, link_max, link_interval).round(2)
        self.links_length = list(itertools.product(lengths_2_check, repeat=(number)))
        self.joints_axis = list(itertools.product(['y', 'z'], repeat=number))
        self.joints = list(itertools.product(['prismatic', 'revolute'], repeat=number))
        self.rpy = list(itertools.product(['0', '-90'], repeat=number))

    def getaxis(self):
        return self.joints_axis

    def getjoints(self):
        return self.joints

    def getlinks(self):
        return self.links_length

    def getrpy(self):
        return self.rpy

    def get_combinations(self,links, joints, axis,rpy):
        '''combine all possible combinations of links and joints'''
        combinations = len(links) * len(joints) * len(axis)*len(rpy)
        return combinations


class Assumptinos (object):
    '''Assumptions that used to reduce the number of manipulators to simulate
    explained details in word files and in the functions    '''

    def __init__(self,length=1):
        self.length=length # the length for the first link

    def assume_1(self,links):
        '''first link is always 0.1 meter'''
        if links[0]== 1.0:
            return True
        return False

    def assume_2(self,axe):
        '''first joint axe is always z axe '''
        if axe =='z':
           return True
        return False

    def assume_3(self,joint_types):
        '''No more than 3 parismatics joints'''
        sum=0
        for j in joint_types:
            if j=='prismatic':
                sum=sum+1
            if sum > 3:
                return False
        return True

    def assume_4(self,joint_types):
        '''first joint is always revolute'''
        if joint_types[0] == 'prismatic':
            return False
        return True

    def assume_5(self,roll):
        '''first joint roll is always 0 '''
        if roll[0] == '-90':
            return False
        return True

    def assume_3_4_count(self,n,counter):
        if n==3:
            counter=counter +  4096
        elif n == 4:
            counter = counter+ 65536
        elif n == 5:
            counter = counter + 1048576
        elif n == 6:
            counter = counter + 16777216
        return counter

    def assume_2_count(self,n,counter):
        if n==3:
            counter=counter + 512
        elif n == 4:
            counter = counter + 4096
        elif n == 5:
            counter = counter + 32768
        elif n == 6:
            counter = counter + 262144
        return counter

    def assume_1_count(self, number, counter):
        if number == 3:
            counter = counter + 8
        elif number == 4:
            counter = counter + 16
        elif number == 5:
            counter = counter + 32
        elif number == 6:
            counter = counter + 64
        return counter


def create_folder(name):
    if not os.path.exists(name):
        os.mkdir(name)
    return name


if __name__ == "__main__":
    tic = datetime.datetime.now()
    combinations = 0
    base_folder = '/media/arl_main/New Volume/'+'urdf_'+str(datetime.datetime.now().date())
    base_folder = create_folder(base_folder)
    assum = Assumptinos()
    d = 0; o = 0; t = 0; z = 0; q = 0; r = 0
    for n in range(3, 7):
        sim = ToSimulate(n)
        links_sim = sim.getlinks()  # create all possible links combinations
        joints_sim = sim.getjoints()  # create all possible joints combinations
        axis_sim = sim.getaxis()      # create all possible axis combinations
        rpy_sim = sim.getrpy()      # create all possible rpy combinations
        path = base_folder+'/DOF_' + str(n) + '/'  # where to save the files
        create_folder(path)
        for i in range(len(joints_sim)):  # run about all the joints combination
            if not assum.assume_4(joints_sim[i]):  # check if the joint is meets the assumption 4
                q=assum.assume_3_4_count(n,q)
                continue
            if not assum.assume_3(joints_sim[i]):  # check if the joint is meets the assumption 3
                d=assum.assume_3_4_count(n,d)
                continue
            joints_path = create_folder(path + str(joints_sim[i])) #create folder with the joints types
            for a in range(len(axis_sim)): #run about all the axis combination
                if not assum.assume_2(axis_sim[a][0]):  # check if the joint is meets the assumption 2
                        o = assum.assume_2_count(n,o)
                        continue
                axis_path = create_folder(joints_path + '/' + str(axis_sim[a]))
                for j in range(len(links_sim)): # run about all the links combination
                    if not assum.assume_1(links_sim[j]):   # check if the joint is meets the assumption 1
                        z = assum.assume_1_count(n,z)
                        continue
                    link_path = create_folder(axis_path + '/' + str(links_sim[j]))
                    for k in range(len(rpy_sim)):  # run about all the rpy combination
                        if not assum.assume_5(rpy_sim[k]):
                            r = r+1
                            continue
                        urdf_name = link_path + '/' + str(rpy_sim[k])
                        if False:  # os.path.isfile(urdf_name + '.urdf.xacro'):
                            continue
                        else:
                            urdf = UrdfClass(links_sim[j], joints_sim[i], axis_sim[a],''.join( rpy_sim[k]))
                            urdf.urdf_write(urdf.urdf_data(), urdf_name)
                            t = t+1
        combinations = sim.get_combinations(links_sim, joints_sim, axis_sim, rpy_sim)+combinations
    toc = datetime.datetime.now()
    delta = (toc-tic).seconds



    print('Pre filtered combinations: ' + str(combinations))
    print('Amount of manipulators that filtered total: ' + str(z+d+o+q+r))
    print('Amount of manipulators after filteting: ' + str(t))
    print('Amount of manipulators that filtered due assum 5: '+str(r))
    print('Amount of manipulators that filtered due assum 4: '+str(q))
    print('Amount of manipulators that filtered due assum 3: '+str(d))
    print('Amount of manipulators that filtered due assum 2: ' + str(o))
    print('Amount of manipulators that filtered due assum 1: ' + str(z))
    print('Time of Run (seconds): ' + str(delta))
    print('Combinations per second: ' + str(1.0*combinations/delta))