'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import threading
import xmlrpc.client
import weakref

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        threading.Thread(target=self.proxy.execute_keyframes, args=(keyframes,)).start()
        # YOUR CODE HERE

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        threading.Thread(target=self.proxy.set_transform, args=(effector_name, transform)).start()
        # YOUR CODE HERE


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self, host='localhost', port=8000):
        self.server = xmlrpc.client.ServerProxy(f'http://{host}:{port}', allow_none=True)
        self.post = PostHandler(self)
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.server.get_angle(joint_name)
        # YOUR CODE HERE
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        return self.server.set_angle(joint_name, angle)
        # YOUR CODE HERE

    def get_posture(self):
        '''return current posture of robot'''
        return self.server.get_posture()
        # YOUR CODE HERE

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        return self.server.execute_keyframes(keyframes)
        # YOUR CODE HERE

    def get_transform(self, name):
        '''get transform with given name
        '''
        return self.server.get_transform(name)
        # YOUR CODE HERE

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        return self.server.set_transform(effector_name, transform)
        # YOUR CODE HERE

if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    print("Posture:", agent.get_posture())
    joint = "LHipYawPitch"
    print(f"Angle of {joint} before:", agent.get_angle(joint))
    agent.set_angle(joint, 0.5)
    print("Angle set to 0.5")

    test_transform = [[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0]]
    agent.post.set_transform("LLeg", test_transform)
    print("Non-blocking set_transform called.")