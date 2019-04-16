import logging
import datetime

class ManipulatorSettings(object):
    '''this class create manipulator settings to enter to the sdf (kinematic formulation) '''
    def __init__(self,links=[],joints=[],joints_axis=[],joint_limit=[],radius=0.0375,max=6 ):
        '''
        :param joints: array of joints types- can be 'revolute' or 'prismatic'
        :param links: array of the links lengths in meters [must be positive float](size of links=size of joints -1)
        :param radius: Cylinder radius of a link in meters
        :param max: maximum number of DOF
        manipulator who have N DOF will have N+1 links when the Nth link will be invisible (sdf format demands it)
        the first link will be the base link, who is always the same(connected to the world and link0  - his joint is limited to 0)
        for n=1:N links will be changed changed
        '''
        if not joints:
            joints = ['revolute', 'prismatic', 'revolute', 'revolute', 'revolute', 'revolute']
        if not joints_axis:
            joints_axis = ['z', 'y', 'y', 'y', 'z', 'y']
        if not links:
            links = [1, 1, 1, 1, 1]
        if not joint_limit:
            joint_limit = [[-6.28319, 6.28319], [-6.28319, 6.28319], [-6.28319, 6.28319], [-3.14159, 0.785398],[-3.14159, 3.14159], [-3.14159, 3.14159]]
        self.max=max #  maximum number of DOF
        self.r=radius # each cylinder radius [m]
        self.dof=len(joints) # number of DOF
        self.init_calc(links,joints,joints_axis,joint_limit) # side calculations
        self.manipulator = [self.links, self.visual_collision, self.joint_data]
        #print(self.manipulator[2][2]['upper'])
        #self.manipulator={'joints':joints,'links':links,'inerM':self.inertia_calc(links)}

    def init_calc(self,links,joints,joints_axis,joint_limit):
        self.calc=[];self.mass=[];self.iner_pose=[]
        self.type=[]; self.axis=[]
        for l in links:# Make calculations for all the links
            self.mass.append(self.mass_calc(l))
            self.calc.append(self.pose_calc(l))          # to fix!!!!!!!!!!!!!!!!!!!!!!!!!
            self.iner_pose.append(self.iner_pose_calc(l))# to fix!!!!!!!!!!!!!!!!!!!!!!!!!
        self.iner_mom = self.inertia_calc(links)
        a=0
        for j in joints: #make calculations for all the joints
            self.type.append('\''+j+'\'')
            self.axis.append(self.axis_calc(joints_axis[a]))
            a=a+1
        ##
        base_link={'link_name': '\'link0\'', 'link_pose': '0 0 0 0 0 0','intertial_pose': '0 0 0 0 0 0', 'Ixx': '1', 'Iyy': '1', 'Izz': '1', 'mass': '4'}
        base_visual={'visual_name': '\'base_link_visual_name\'', 'collision_name': '\'base_link_collision\'','link_length': '0.1', 'radius': '0.0375'}
        base_joint={'name': '\'joint0\'', 'type': '\'revolute\'', 'child_link': 'link0','parent_link': 'world', 'axis_xyz': '0 0 1', 'lower': '0','upper': '0'}
        last_link={'link_name': '\'link'+str(self.dof)+'\'', 'link_pose': '0.81725 0.10915 -0.005491 3.14159 3.58979e-09 3.14159','intertial_pose': '0 0 0 0 0 0', 'Ixx': '1', 'Iyy': '1', 'Izz': '1', 'mass': '0.1879'}
        last_visual={'visual_name': '\'last_link_visual_name\'', 'collision_name': '\'last_link_collision\'','link_length': '0.1', 'radius': '0.0375'}

        self.links = [{'link_name': '\'link' + str(x) + '\'', 'link_pose': self.calc[x-1],'intertial_pose': self.iner_pose[x-1], 'Ixx': str(self.iner_mom[x-1][0]), 'Iyy': str(self.iner_mom[x-1][1]), 'Izz': str(self.iner_mom[x-1][2]), 'mass': str(self.mass[x-1])}
                      for x in range(1,self.dof)]
        self.visual_collision = [{'visual_name': '\'link' + str(x) + '_visual_name\'', 'collision_name': '\'link' + str(x) + '_collision\'','link_length': str(links[x-1]), 'radius': str(self.r)}
                      for x in range(1,self.dof)]
        self.joint_data = [{'name': '\'joint' + str(x) + '\'', 'type': self.type[x-1], 'child_link': 'link' + str(x),'parent_link': 'link' + str(x - 1), 'axis_xyz': self.axis[x-1], 'lower': str(joint_limit[x-1][0]), 'upper': str(joint_limit[x-1][1])}
                      for x in range(1,self.dof+1)]

        self.links=[base_link]+self.links+[last_link]
        self.visual_collision = [base_visual] + self.visual_collision + [last_visual]
        self.joint_data = [base_joint] +self.joint_data

    def set_joints(self,joints):
        if len(joints) == 0:
            logging.warning('Number of joints is 0 ' + str(datetime.datetime.now()))  # will print a message to the console
        elif len(joints) > self.max:
            logging.warning('Number of joints is bigger than ' + str(self.max) + '! ' + str(datetime.datetime.now()))  # will print a message to the console
        elif not self.check_joints(joints):
            logging.warning('Wrong joint type! ' + str(datetime.datetime.now()))  # will print a message to the console
        else:
            self.manipulator['joints']=joints

    def get_joints(self):
        return self.manipulator['joints']

    def get_num_joints(self):
        return len(self.manipulator['joints'])

    def set_links(self,links):
        if len(links) == len(self.manipulator['joints']):
            self.manipulator['links']=links
        else:
            logging.warning('Number of joints and links does not match ' + str(datetime.datetime.now()))  # will print a message to the console

    def get_links(self):
        return self.manipulator['links']

    def check_joints(self,joints):
        ''' check if the joints types that inserted are linear or revolute'''
        for i in joints:
            if i!= "revolute" or i!="prismatic":
                return False
        return True

    def inertia_calc(self,links):
        '''This function calculate moments of inertia to each link - all joints are  Solid cylinder of radius r, height h and mass m'''
        inerM=[]
        j=0
        for i in links:
            Ixx = float('%.2f' %((self.mass[j]/12.0)*(3*self.r**2 + i**2)))
            Iyy = Ixx
            Izz = float('%.2f' % (0.5*self.mass[j]*self.r**2))
            #inerM.append(self.array2string([Ixx, Iyy, Izz, 0, 0, 0]) )# Ixx, Iyy, Izz, Ixy, Ixz, Iyz
            inerM.append([Ixx, Iyy, Izz])  # Ixx, Iyy, Izz,
            j=j+1
        return inerM

    def mass_calc(self,length=1):
        '''The mass is calculated according to UR5 data
        We know the weight of cylinder in spcecific length and specific radius and calculate according it
        '''
        specific_length = 1.0  # [meters]
        specific_mass = 4.0  # [kg] mass of cylinder at the specified radius and length
        mass=(length/specific_length)* specific_mass
        return mass

    def pose_calc(self,l):
        '''Calculate where to position the links'''
        if l>3:
            return '0.5 0.2 0.3 0 0 0'
        return '0 0.1 0 0 0 0.7'

    def iner_pose_calc(self,l):
        '''Calculate the inertail pose'''
        if l>3:
            return '0.05 0.02 0.03 0 0 0'
        return '0 0.41 0 0 0 0.07'

    def axis_calc(self,axe):
        if axe == 'x':
            return '1 0 0'
        elif axe == 'y':
            return '0 1 0'
        elif axe == 'z':
            return '0 0 1'
        else:
            logging.warning('wrong axe input.'+ axe + ' entered. returning [0 0 0] ' + str(datetime.datetime.now()))  # will print a message to the console
            return '0 0 0'

    def array2string(self,array):
        '''Make string from array'''
        string=''
        for i in range(len(array)):
            string = string + ' ' + str(array[i])
        return string


#test = ManipulatorSettings([5,1],['prismatic','revolute','prismatic'],['x','y','z'])
#print(test.manipulator)
#print(test.get_num_joints())
#test.set_joints(['prismatic','prismatic','prismatic','prismatic','prismatic','linear'])
#print(test)
# test.set_joints([])
# print(test.get_joints())
#
# print(test.get_links())
# test.set_links([1,2,3])
# print(test.get_links())
# test.set_links([6])
# print(test.get_links())
#print(test.inertia_calc([2,4]))
#print(test)



