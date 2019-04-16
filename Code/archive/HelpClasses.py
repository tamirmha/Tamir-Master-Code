import datetime
from logging import warning

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
            links = [1, 1, 1, 1, 1,1]
        #if not joint_limit:
        #    joint_limit = [[-3.14159, 3.14159],[-3.14159, 3.14159],[-3.14159, 3.14159],[-3.14159, 3.14159],[-3.14159, 3.14159], [-3.14159, 3.14159]]
        #self.max=max #  maximum number of DOF
        #self.r=radius # each cylinder radius [m]
        self.dof=len(joints) # number of DOF
        self.links=links
        self.joints = joints
        self.joint_axis=joints_axis
        self.init_calc(self.joints,self.joint_axis)
        self.manipulator = [self.links, self.joints,self.axis]

    def init_calc(self,joints,joints_axis):
        self.type=[]; self.axis=[]
        #self.mass=[]; self.calc=[]
        a=0
        for j in joints: #make calculations for all the joints
            self.type.append('\''+j+'\'')
            self.axis.append(self.axis_calc(joints_axis[a]))
            a=a+1

    def axis_calc(self,axe):
        if axe == 'x':
            return '1 0 0'
        elif axe == 'y':
            return '0 1 0'
        elif axe == 'z':
            return '0 0 1'
        else:
            warning('wrong axe input.'+ axe + ' entered. returning [0 0 0] ' + str(datetime.datetime.now()))  # will print a message to the console
            return '0 0 0'

    '''def array2string(self,array):
        ''Make string from array''
        string=''
        for i in range(len(array)):
            string = string + ' ' + str(array[i])
        return string
    '''

class UrdfClass(object):
    '''this class create sdf files
    sdf- kinematics presentation of manipulators
    joints- the manipulator joints in this specific order: rot - Rotational joint
                                                           lin - Linear joint
            the length of joints is the number of DOF
    links - array of lengths of the links [meters]
    inerM- inertia moments of the links -
    '''

    def __init__(self, manipulator):
        self.manipulator = manipulator
        self.links_number = len(manipulator[1])
        self.links = manipulator[0]
        self.joint_data = manipulator[1]
        self.axis = manipulator[2]

    def urdf_data(self):
        head='''<?xml version="1.0"?>
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

        inertia_parameters='''
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
        base_link='''
        
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
             data = data+ self.joint_create(i+1) + self.link_create(i+1)

        tail='''    
    <joint name="${prefix}ee_fixed_joint" type="fixed">
      <parent link="${prefix}link'''+str(self.links_number)+'''" />
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

        txt = head +inertia_parameters+ base_link+data+ tail
        return txt

    def link_create(self, n):
        '''link- data about specific link. it buit from :
                *inertia - inertail data of the link -Mass, moment of inertia, pose in the frame
                *collision - the collision properties of a link.
                *visual - > the visual properties of the link. This element specifies the shape of the object (box, cylinder, etc.) for visualization purposes
                *velocity_decay - exponential damping of the link's velocity'''
        linkname = 'link' + str(n)
        link=''
        if n==1:
            link =link+ '''<!--  link 1  -->
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
            link = link +'''<!-- link '''+str(n)+'''	-->
    <link name="${prefix}'''+linkname+'''">
      <visual>
		<origin xyz="0 0 ${'''+linkname+'''_length / 2}" rpy="0 0 0" />
        <geometry>
			<cylinder radius="${'''+linkname+'''_radius}" length="${'''+linkname+'''_length}"/> 
        </geometry>
      </visual>
      <collision>
	    <origin xyz="0 0 ${'''+linkname+'''_length / 2 }" rpy="0 0 0" />
        <geometry>
		<cylinder radius="${'''+linkname+'''_radius}" length="${'''+linkname+'''_length}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertial radius="${'''+linkname+'''_radius}" length="${'''+linkname+'''_length}" mass="${'''+linkname+'''_mass}">
        <origin xyz="0.0 0.0 ${'''+linkname+'''_length / 2 }" rpy="0 0 0" />
      </xacro:cylinder_inertial>
    </link>'''
        return link

    def calc_origin(self,axe,length,radius,n):
        if n==2:
            return '0.0 ${link2_radius * 2.0} ${link1_length /2}'
        elif n==3:
            return '0.0 ${link3_radius*2.0} ${link2_length}'
        elif n==4:
            return '0 -${(link4_radius) * 2 } ${link3_length}'
        elif n==5:
            return '0 0 ${link4_length+link5_radius}'
        elif n==6:
            return '0 0 ${link5_length }'

    def calc_rpy(self,axe,length,radius,n):
        if n==2:
            return '0 0 0'
        elif n==3:
            return '0 0 0'
        elif n==4:
            return '0 0 0'
        elif n==5:
            return '${-pi/2} 0 0'
        elif n==6:
            return '0 0 0'

    def joint_create(self, n):
        jointname='joint' + str(n)
        orgin=self.calc_origin(1,1,1,n)
        rpy=self.calc_rpy(1,1,1,n)
        joint='\n<xacro:property name="'+jointname+'_type" value="'+self.joint_data[n-1]+'"/>\n'\
              '<xacro:property name="'+jointname+'_axe" value="'+self.axis[n-1]+'"/>\n'\
        '<xacro:property name="link'+str(n)+'_length" value="'+str(self.links[n-1])+'"/>\n'

        if n==1:
            joint=joint+'''<!--  joint 1	-->
    <joint name="${prefix}joint1" type="${joint1_type}">
      <parent link="${prefix}base_link" />
      <child link="${prefix}link1" />
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
      <axis xyz="${joint1_axe}"/>
	  <xacro:joint_limit joint_type="${joint1_type}" link_length="${link1_length}"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>
    '''
        else:
            joint=joint+'''<!--  joint '''+str(n) +'''	-->
    <joint name="${prefix}'''+jointname+'''" type="${'''+jointname+'''_type}">
      <parent link="${prefix}link'''+str(n-1)+'''"/>
      <child link="${prefix}link'''+str(n)+'''" />
      <origin xyz="'''+orgin+'''" rpy="'''+rpy+'''"/>
      <axis xyz="${'''+jointname+'''_axe}"/>
	  <xacro:joint_limit joint_type="${'''+jointname+'''_type}" link_length="${link'''+str(n)+'''_length}"/>
      <dynamics damping="0.0" friction="0.0"/>
    </joint>
    '''
        return joint

    def urdf_write(self, data, filename=str(datetime.datetime.now().minute)):
        file = open(filename + '.urdf.xacro', 'w')
        file.write(data)
        file.close()

test = ManipulatorSettings([0.15,0.5,0.4,0.07,0.1,0.02], ['revolute', 'revolute', 'revolute','revolute','revolute','revolute'], ['z', 'y', 'y', 'y', 'y','z'])
#print(test.axis)
#print(test.manipulator[0])
#print(len(test.manipulator[0])-1)
urdf = UrdfClass(test.manipulator)
#print (urdf.urdf_data())
urdf.urdf_write(urdf.urdf_data(), 'ur5')
#print (test.manipulator)


params = '''< xacro: macro name = "ur5_robot" params = "prefix joint_limited" >\n
'< xacro: property name = "joint1_type" value = "'+self.joint_data[0]+'" / >\n
'< xacro: property name = "joint2_type"  value = "'+self.joint_data[1]+'" / >\n
'< xacro: property name = "joint3_type" value = "'+self.joint_data[2]+'" / >\n
'< xacro: property name = "joint4_type"  value = "'+self.joint_data[3]+'" / >\n
'< xacro: property name = "joint5_type" value = "'+self.joint_data[4]+'" / >\n
'< xacro: property name = "joint6_type"  value = "'+self.joint_data[5]+'" / >\n\n

'< xacro: property name = "joint1_axe" value = "0 0 1" / >\n
'< xacro: property name = "joint2_axe" value = "0 1 0" / >\n
'< xacro: property name = "joint3_axe" value = "0 1 0" / >\n
'< xacro: property name = "joint4_axe" value = "0 1 0" / >\n
'< xacro: property name = "joint5_axe" value = "0 1 0" / >\n
'< xacro: property name = "joint6_axe" value = "0 0 1" / >\n

'< xacro: property name = "base_length" value = "0.05" / >\n
'< xacro: property name = "link1_length" value = "0.15" / >\n
'< xacro: property name = "link2_length" value = "0.50" / >\n
'< xacro: property name = "link3_length" value = "0.40" / >\n
'< xacro: property name = "link4_length" value = "0.07" / >\n
'< xacro: property name = "link5_length" value = "0.10" / >\n
'< xacro: property name = "link6_length" value = "0.02" / >\n'''