import datetime
import ManipulatorSettings as ms

class SdfClass(object):
    '''this class create sdf files
    sdf- kinematics presentation of manipulators
    joints- the manipulator joints in this specific order: rot - Rotational joint
                                                           lin - Linear joint
            the length of joints is the number of DOF
    links - array of lengths of the links [meters]
    inerM- inertia moments of the links -
    '''
    def __init__(self,manipulator):
        self.manipulator=manipulator
        self.links_number=3
        self.links =manipulator[0]
        self.visual_collision=manipulator[1]
        self.joint_data =manipulator[2]
        print self.joint_data[3]

    def sdf_data(self):
        '''I built the sdf file from 4 part
            head- formal data that common to all files
            link- data about specific link. it buit from :
                *inertia - inertail data of the link -Mass, moment of inertia, pose in the frame
                *collision - the collision properties of a link.
                *visual - > the visual properties of the link. This element specifies the shape of the object (box, cylinder, etc.) for visualization purposes
                *velocity_decay - exponential damping of the link's velocity
            joint -data about specific joint. it buit from :
                    *axis-The joint axis specified in the parent model frame. This is the axis of rotation for revolute joints,
                         the axis of translation for prismatic joints. The axis is currently specified in the parent model frame of reference,
                    *xyz- Represents the x,y,z components of a vector. The vector should be normalized
                    *limit-specifies the limits of this joint
                    *lower\upper-radians for revolute joints, meters for prismatic joints
                    *effort-An attribute for enforcing the maximum joint effort applied by Joint::SetForce.  Limit is not enforced if value is negative
                    *velocity-(not implemented) An attribute for enforcing the maximum joint velocity
                    *damping -The physical velocity dependent viscous damping coefficient of the joint
            tail- formal data that common to all files
        '''
        head='<?xml version=\'1.0\'?> \n'\
        '<sdf version=\'1.4\'> \n' \
        ' <model name=\'Manipulator\'> \n'
        data=''
        for i in range(self.links_number):
            data=data+self.link_create(i)+self.joint_create(i)
        tail='\n </model>   \n</sdf>'
        txt=head + data+ tail
        return txt

    def link_create(self,n):
        '''link- data about specific link. it buit from :
                *inertia - inertail data of the link -Mass, moment of inertia, pose in the frame
                *collision - the collision properties of a link.
                *visual - > the visual properties of the link. This element specifies the shape of the object (box, cylinder, etc.) for visualization purposes
                *velocity_decay - exponential damping of the link's velocity'''
        visual_collision=self.visual_collision[n]
        Inertial=self.links[n]
        inertia='   <link name=' + Inertial['link_name'] + '>\n    <pose>' + Inertial['link_pose'] + '</pose>\n' \
        '    <inertial> \n' \
        '      <pose>' + Inertial['intertial_pose'] + '</pose>\n' \
        '      <mass>' + Inertial['mass'] + '</mass>\n'\
        '      <inertia>\n'\
        '        <ixx>' + Inertial['Ixx'] + '</ixx>\n'\
        '        <iyy>' + Inertial['Iyy'] + '</iyy>\n'\
        '        <izz>' + Inertial['Izz'] + '</izz>\n        <ixy>0</ixy>\n        <ixz>0</ixz>\n        <iyz>0</iyz>\n'\
        '      </inertia>\n'\
        '    </inertial>\n'
        collision = '    <collision name = ' + visual_collision['collision_name'] + '>\n' \
        '     <geometry> \n       <cylinder> \n         <radius>' + visual_collision['radius']  + '</radius>\n         <length>' + visual_collision['link_length']  + '</length>\n       </cylinder>\n    </collision>\n'
        visual = '    <visual name = ' + visual_collision['visual_name'] + '>\n' \
                 '     <geometry> \n       <cylinder> \n         <radius>' + visual_collision['radius'] + '</radius>\n         <length>' + visual_collision['link_length'] + '</length>\n       </cylinder>\n    </visual>\n'
        velocity_decay = '    <velocity_decay>\n         <linear>0</linear> \n        <angular>0</angular> \n    </velocity_decay>\n   </link>\n'
        link = inertia + collision + visual + velocity_decay
        return link

    def joint_create(self,n):
        '''joint -data about specific joint. it buit from :
                    *axis-The joint axis specified in the parent model frame. This is the axis of rotation for revolute joints,
                         the axis of translation for prismatic joints. The axis is currently specified in the parent model frame of reference,
                    *xyz- Represents the x,y,z components of a vector. The vector should be normalized
                    *limit-specifies the limits of this joint
                    *lower\upper-radians for revolute joints, meters for prismatic joints
                    *effort-An attribute for enforcing the maximum joint effort applied by Joint::SetForce.  Limit is not enforced if value is negative
                    *velocity-(not implemented) An attribute for enforcing the maximum joint velocity
                    *damping -The physical velocity dependent viscous damping coefficient of the joint'''
        joint_data = self.joint_data[n]
        effort='10'       # doesn't change in ur5 sdf
        velocity='3.14159'# doesn't change in ur5 sdf
        damping='10'      # doesn't change in ur5 sdf
        joint = '   <joint name=' + joint_data['name'] + ' type=' + joint_data['type'] + '>\n' \
                '     <child>' + joint_data['child_link'] + '</child>\n' \
                '     <parent>' + joint_data['parent_link'] + '</parent>\n     <axis>\n' \
                '       <xyz>' + joint_data['axis_xyz'] + '</xyz>\n       <limit>\n' \
                '           <lower>' + joint_data['lower'] + '</lower>\n' \
                '           <upper>' + joint_data['upper'] + '</upper>\n' \
                '           <effort>' + effort + '</effort>\n' \
                '           <velocity>' + velocity + '</velocity>\n' \
                '       </limit>\n       <dynamics>\n          <damping>' + damping + '</damping>\n       </dynamics>\n     </axis>\n   </joint> \n'
        return joint

    def sdf_write(self,data,filename=str(datetime.datetime.now().minute)):
        file = open(filename+'.sdf', 'w')
        file.write(data)
        file.close()

test = ms.ManipulatorSettings([5,1],['prismatic','revolute','prismatic'],['x','y','z'])
sdf=SdfClass(test.manipulator)

sdf.sdf_write(sdf.sdf_data(),'rrr')
#test.dof
#print(test.manipulator[1])
