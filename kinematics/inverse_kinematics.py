'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
import numpy as np
from numpy.matlib import identity


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        desired_pos = np.array(transform)[3, :3].astype(float)
        chain = self.chains[effector_name]
        angles = np.zeros(len(chain))
        delta = 1e-4
        alpha = 0.5


        # numerisches IK-Verfahren (Jacobian-Methode)
        for _ in range(60):
            # aktuelle Winkel in ein joints-Dict übernehmen
            joints = {name: 0.0 for name in self.joint_names}
            for i, joint in enumerate(chain):
                joints[joint] = angles[i]

            # aktuelle Fußposition per Vorwärtskinematik bestimmen
            self.forward_kinematics(joints)
            current_pos = np.array(self.transforms[chain[-1]])[:3, 3]
            err = desired_pos - current_pos
            if np.linalg.norm(err) < 1e-4:
                break  # Fehler ausreichend klein

            # Jacobian numerisch berechnen
            J = np.zeros((3, len(chain)))
            for j in range(len(chain)):
                pert_angles = angles.copy()
                pert_angles[j] += delta
                pert_joints = {name: 0.0 for name in self.joint_names}
                for k, joint in enumerate(chain):
                    pert_joints[joint] = pert_angles[k]
                self.forward_kinematics(pert_joints)
                new_pos = np.array(self.transforms[chain[-1]])[:3, 3]
                J[:, j] = (new_pos - current_pos) / delta
            # Winkelkorrektur über Pseudoinverse des Jacobians
            angles += alpha * np.linalg.pinv(J) @ err

        # berechnete Winkel in joint_angles ablegen
        joint_angles = list(angles)
        # YOUR CODE HERE
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        # IK aufrufen
        angles = self.inverse_kinematics(effector_name, transform)
         # passende Gelenknamen für das gewählte Bein
        if effector_name == 'LLeg':
            joints = ['LHipYawPitch', 'LHipRoll', 'LHipPitch',
                      'LKneePitch', 'LAnklePitch', 'LAnkleRoll']
        else:
            joints = ['RHipYawPitch', 'RHipRoll', 'RHipPitch',
                      'RKneePitch', 'RAnklePitch', 'RAnkleRoll']

        # Keyframe-Listen aufbauen
        names = []
        times = []
        keys = []
        for jname, ang in zip(joints, angles):
            names.append(jname)
            times.append([1.0])          # ein Zielzeitpunkt bei 1.0 s
            keys.append([ang])           # Winkel als Liste

        # Ergebnisse abspeichern
        self.keyframes = (names, times, keys)

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = -0.26
    agent.set_transforms('LLeg', T)
    agent.run()
