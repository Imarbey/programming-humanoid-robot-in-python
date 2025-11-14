'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        names, times, keys =keyframes 
        if not hasattr(self,"_startTime"): 
            self._startTime = float(getattr(perception, "time", 00))
        t = float(getattr(perception, "time", 00)) - self._startTime


        def parse_key(entry): 
            if isinstance(entry, (list, tuple)): 
                a =float(entry[0])
                h_prev = entry[1] if len(entry) > 1 else (0, 0.0, 0.0) 
                h_next = entry[2] if len(entry) > 2 else (0, 0.0,0.0)
                return a, h_prev, h_next 
            return float(entry), (0, 0.0, 0.0), (0, 0.0, 0.0)
        
        for i, name in enumerate(names): 
            ts, ks = times[i], keys[i]
            if not ts: 
                continue 
            
            if t <= ts[0]:
                target_joints[name] = parse_key(ks[0])[0]
                continue 
           
            if t >= ts[-1]:
                target_joints[name] = parse_key(ks[-1])[0]
                continue

            seg = next(j for i in range(len(ts)-1) if ts[j] <= t< ts[j+1])
            t0, t1 =ts[seg], ts[seg+1]   
            a0, _, h0_next =parse_key(ks[seg])
            a1, h1_prev, _ =parse_key(ks[seg+1])

            u = (t-t0)/(t1-t0)
            omu =1.0-u 
            p0=a0
            p1=a0 + float(h0_next[2])
            p2 =a1+ float(h1_prev[2])
            p3=a1
            
            angle =(omu**3)*p0 +3*(omu**2)*u*p1 +3*omu*(u**2)*p2+(u**3)*p3 
            target_joints[name]=float(angle)



        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
