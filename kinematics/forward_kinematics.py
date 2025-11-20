'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
import numpy as np

from recognize_posture import PostureRecognitionAgent


class ForwardKinematicsAgent(PostureRecognitionAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = { 
            'Head': ['HeadYaw', 'HeadPitch'],
             # YOUR CODE HERE
            'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw'],
            'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw'],
            'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
            'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
        
        }

        # Längen und Offsets
        self.NeckOffsetZ     = 0.12650
        self.ShoulderOffsetY = 0.09800
        self.ShoulderOffsetZ = 0.10000
        self.UpperArmLength  = 0.09000
        self.LowerArmLength  = 0.05055
        self.HipOffsetY      = 0.05000
        self.HipOffsetZ      = 0.08500
        self.ThighLength     = 0.10000
        self.TibiaLength     = 0.10274
        self.FootHeight      = 0.04511

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)
    
    def _trans(self, dx, dy, dz):
        T = identity(4)
        T[0, 3] = dx
        T[1, 3] = dy
        T[2, 3] = dz
        return T
    
    def _rotx(self, angle):
        c, s = np.cos(angle), np.sin(angle)
        return matrix([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]])

    def _roty(self, angle):
        c, s = np.cos(angle), np.sin(angle)
        return matrix([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]])

    def _rotz(self, angle):
        c, s = np.cos(angle), np.sin(angle)
        return matrix([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
       
        # Kopf
        if joint_name == 'HeadYaw':
            T = self._trans(0, 0, self.NeckOffsetZ )@ self._rotz(joint_angle)
        elif joint_name == 'HeadPitch':
            T = self._roty(joint_angle)

        # linker Arm
        elif joint_name == 'LShoulderPitch':
            T = self._trans(0,self.ShoulderOffsetY,self.ShoulderOffsetZ) @self._roty(joint_angle)
        elif joint_name == 'LShoulderRoll':
            T = self._rotz(joint_angle)
        elif joint_name == 'LElbowYaw':
            T = self._trans(self.UpperArmLength, 0, 0) @ self._rotx(joint_angle)
        elif joint_name == 'LElbowRoll':
            T = self._rotz(joint_angle)
        elif joint_name == 'LWristYaw':
            T = self._trans(self.LowerArmLength, 0, 0) @ self._rotx(joint_angle)

        # rechter Arm
        elif joint_name == 'RShoulderPitch':
            T = self._trans(0, -self.ShoulderOffsetY, self.ShoulderOffsetZ) @ self._roty(joint_angle)
        elif joint_name == 'RShoulderRoll':
            T = self._rotz(joint_angle)
        elif joint_name == 'RElbowYaw':
            T = self._trans(self.UpperArmLength, 0, 0) @ self._rotx(joint_angle)
        elif joint_name == 'RElbowRoll':
            T = self._rotz(joint_angle)
        elif joint_name == 'RWristYaw':
            T = self._trans(self.LowerArmLength, 0, 0) @ self._rotx(joint_angle)

        # Hüft-Jaw-Pitch
        elif joint_name == 'LHipYawPitch':
            T = self._trans(0,  self.HipOffsetY, -self.HipOffsetZ) @ self._rotz(joint_angle)
        elif joint_name == 'RHipYawPitch':
            T = self._trans(0, -self.HipOffsetY, -self.HipOffsetZ) @ self._rotz(joint_angle)

        # linkes Bein
        elif joint_name == 'LHipRoll':
            T = self._rotx(joint_angle)
        elif joint_name == 'LHipPitch':
            T = self._roty(joint_angle)
        elif joint_name == 'LKneePitch':
            T = self._trans(0, 0, -self.ThighLength) @ self._roty(joint_angle)
        elif joint_name == 'LAnklePitch':
            T = self._trans(0, 0, -self.TibiaLength) @ self._roty(joint_angle)
        elif joint_name == 'LAnkleRoll':
            T = self._rotx(joint_angle)   

        # rechtes Bein
        elif joint_name == 'RHipRoll':
            T = self._rotx(joint_angle)
        elif joint_name == 'RHipPitch':
            T = self._roty(joint_angle)
        elif joint_name == 'RKneePitch':
            T = self._trans(0, 0, -self.ThighLength) @ self._roty(joint_angle)
        elif joint_name == 'RAnklePitch':
            T = self._trans(0, 0, -self.TibiaLength) @ self._roty(joint_angle)
        elif joint_name == 'RAnkleRoll':
            T = self._rotx(joint_angle)     


        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T @ Tl

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
