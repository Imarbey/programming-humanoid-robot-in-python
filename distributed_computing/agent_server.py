'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from xmlrpc.server import SimpleXMLRPCServer


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self, host='localhost', port=8000):
        super(ServerAgent, self).__init__()
        self.server = SimpleXMLRPCServer((host, port), allow_none=True)
        self.server.register_instance(self)
        print(f"Server started at {host}:{port}")

    def run(self):
        self.server.serve_forever()

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.get_joint_angle(joint_name)
        # YOUR CODE HERE
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        self.set_joint_target(joint_name, angle)
        # YOUR CODE HERE

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.get_current_name()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.angle_interpolation(keyframes)
        return True

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.get_transform_matrix(name).tolist()

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        return self.inverse_kinematics(effector_name, transform)

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

