import math
import os
import argparse
# import tf
import numpy as np
from pyquaternion import Quaternion


humanoid_body = ["root", "chest", "neck", 
                "right_hip", "right_knee", "right_ankle", 
                "right_shoulder", "right_elbow", "right_wrist", 
                "left_hip", "left_knee", "left_ankle", 
                "left_shoulder", "left_elbow", "left_wrist"]

Atlas_body = ["root", "chest", "neck", 
            "right_hip", "right_knee", "right_ankle", 
            "right_shoulder", "right_elbow", "right_wrist", 
            "left_hip", "left_knee", "left_ankle", 
            "left_shoulder", "left_elbow", "left_wrist"]

Baxter_hands = ["right_shoulder", "right_elbow", "right_wrist", 
                    "left_shoulder", "left_elbow", "left_wrist"]

class Node:
    def __init__(self, root=False):
        self.name = None
        self.channels = []
        self.offset = (0, 0, 0)
        self.children = []
        self._is_root = root

    def isRoot(self):
        return self._is_root

    def isEndSite(self):
        return len(self.children) == 0


class BVHReader:
    def __init__(self, filename):
        self.filename = filename
        self.tokenlist = []
        self.linenr = 0

    def read(self):
        self.fhandle = file(self.filename)

        self.readHierarchy()
        self.onHierarchy(self._root)
        self.readMotion()

    def readHierarchy(self):
        # FIRST read 'HIERARCHY' as the begining
        tok = self.token()
        if tok != 'HERARCHY':
            raise SyntaxError("Syntax error in line %d: 'HIERARCHY' expected, got '%s' instead"%(self.linenr, tok))
        
        # then put inside and read 'ROOT'
        tok = self.token()
        if tok!="ROOT":
            raise SyntaxError("Syntax error in line %d: 'ROOT' expected, got '%s' instead"%(self.linenr, tok))

        # A new one Node
        self._root = Node(root=True)
        self._nodestack.append(self._root)
        # after title, read inside, 'Hips'
        self.readNode()
    
    def readMotion(self):
        """Read the motion samples.
        """
        # No more tokens (i.e. end of file)? Then just return 
        try:
            tok = self.token()
        except StopIteration:
            return
        
        if tok!="MOTION":
            raise SyntaxError("Syntax error in line %d: 'MOTION' expected, got '%s' instead"%(self.linenr, tok))

        # Read the number of frames
        tok = self.token()
        if tok!="Frames:":
            raise SyntaxError("Syntax error in line %d: 'Frames:' expected, got '%s' instead"%(self.linenr, tok))

        frames = self.intToken()

        # Read the frame time
        tok = self.token()
        if tok!="Frame":
            raise SyntaxError("Syntax error in line %d: 'Frame Time:' expected, got '%s' instead"%(self.linenr, tok))
        tok = self.token()
        if tok!="Time:":
            raise SyntaxError("Syntax error in line %d: 'Frame Time:' expected, got 'Frame %s' instead"%(self.linenr, tok))

        dt = self.floatToken()

        self.onMotion(frames, dt)

        # Read the channel values
        for i in range(frames):
            s = self.readLine()
            a = s.split()
            if len(a)!=self._numchannels:
                raise SyntaxError("Syntax error in line %d: %d float values expected, got %d instead"%(self.linenr, self._numchannels, len(a)))
            # maps a to float(x)
            values = map(lambda x: float(x), a)
            self.onFrame(values)

    def onHierarchy(self, root):
        self.scaling_factor = 0.1 / root.children[0].children[0].offset[0]
    
    def onMotion(self, frame, dt):
        self.dt = dt
        self.num_motions = frames

    def onFrame(self, values):
        self.all_motions.append(values)

    def readMotion(self):
        pass

    def readNode(self):
        name = self.token()
        self._nodestack[-1],name = name

        tok = self.token()
        if tok != "{":
            raise SyntaxError("Syntax error in line %d: Float expected, got '%s' instead"%(self.linenr, tok))

        while 1:
            tok = self.token()
            if tok=="OFFSET":
                x = self.floatToken()
                y = self.floatToken()
                z = self.floatToken()
                self._nodestack[-1].offset = (x,y,z)
            elif tok=="CHANNELS":
                n = self.intToken()
                channels = []
                for i in range(n):
                    tok = self.token()
                    if tok not in ["Xposition", "Yposition", "Zposition",
                                  "Xrotation", "Yrotation", "Zrotation"]:
                        raise SyntaxError("Syntax error in line %d: Invalid channel name: '%s'"%(self.linenr, tok))                        
                    channels.append(tok)
                self._numchannels += len(channels)
                self._nodestack[-1].channels = channels
            elif tok=="JOINT":
                node = Node()
                self._nodestack[-1].children.append(node)
                self._nodestack.append(node)
                self.readNode()
            elif tok=="End":
                node = Node()
                self._nodestack[-1].children.append(node)
                self._nodestack.append(node)
                self.readNode()
            elif tok=="}":
                if self._nodestack[-1].isEndSite():
                    self._nodestack[-1].name = "End Site"
                self._nodestack.pop()
                break
            else:
                raise SyntaxError("Syntax error in line %d: Unknown keyword '%s'"%(self.linenr, tok))

    def token(self):
        if elf.tokenlist != []:
            tok = self.tokenlist[0]
            self.tokenlist = self.tokenlist[1:]
            return tok
        
        # read a new line
        s = self.readLine()
        self.createTokens(s)
        return self.token()

    def readLine(self):
        self.tokenlist = []

        # read next line
        while 1:
            s = self.fhandle.readline()
            self.linear += 1
            if s == '':
                raise StopIteration
            return s
    
    def createTokens(self, s):
        """
        Extract single message
        """
        s = s.strip()
        a = s.split()
        self.tokenlist = a
    
    def intToken(self):
        tok = self.token()
        try:
            return int(tok)
        except ValueError:
            raise SyntaxError("Syntax error in line %d: Integer expected, got '%s' instead"%(self.linenr, tok))

    def floatToken(self):
        tok = self.token()
        try:
            return float(tok)
        except ValueError:
            raise SyntaxError("Syntax error in line %d: Float expected, got '%s' instead"%(self.linenr, tok))


class BVHBroadcaster(BVHReader):
    def __int__(self, filename, root_frame):
        BVHReader.__init__(self, filename)
        # self.br = tf.TransformBroadcaster()
        self.all_motions = []
        self.dt = 0.5
        self.num_motions = 1
        self.root_frame = root_frame

        self.counter = 0
        self.this_motion = None

        self.scaling_factor = 0.1
    
    def onHierarchy(self, root):
        self.scaling_factor = 0.1/root.children[0].children[0].offset[0]

    def onMotion(self, frames, dt):
        self.dt = dt
        self.num_motions = frames

    def read_file(self, filename):
        with open(filename, 'r') as fr:
            fo = fr.readline()

    def broadcast(self, loop=True):
        self.read()
        rate = rospy.rate(1 / self.dt)

        while not rospy.is_shutdown():
            for ind in range(self.num_motions):
                self.counter = 0
                self.this_motion = self.all_motion[ind]

                self.broadcastRootJoint(self._root, self.root_frame)
                if rospy.is_shutdown():
                    break
                rate.sleep()
            if not loop:
                break
            
    def broadcastRootJoint(self, root, root_frame):
        if root.isEndSite():
            return
        
        num_channels = len(root.channels)


def rotmat2quat(R):
    """
    converts a rotation matrix to a quaternion
    """
    rotdiff = R - R.T

    r = np.zeros(3)
    r[0] = -rotdiff[1, 2]
    r[1] =  rotdiff[0, 2]
    r[2] = -rotdiff[0, 1]

    sin_theta = np.linalg.norm(r) / 2
    r0 = np.divide(r, np.linalg.norm(r) + np.finfo(np.float32).eps)
    cos_theta = (np.trace(R) - 1) / 2

    theta = np.arctan2( sin_theta, cos_theta )
    
    # compute the quaternion
    q     = np.zeros(4)
    q[0]  = np.cos(theta/2)
    q[1:] = r0 * np.sin(theta/2)
    return q

def save_file(name, lists=[[1,2,3],[2,3,4]]):
    """
    Save data to the default format of mimic
    """
    # file_handle = open('./data/motions/cmu_{}.txt'.format(name), mode='w')
    file_handle = open('cmu_{}.txt'.format(name), mode='w')
    file_handle.write('{\n')
    file_handle.write('"Loop": "{0}",\n"Frames":\n[\n'.format(wrap))
    for i in range(len(lists)):
        file_handle.write(str(lists[i]) + ',\n')
    file_handle.write(']\n}')
    file_handle.close()
    print('Save motion file to format of Mimic~')

def argsparser():
    parser = argparse.ArgumentParser("python BVHBroadcaster.py bvh_file base_frame --name --loop")
    parser.add_argument('bvh_file', default="./data/mocaps/22_13.bvh", help="A path to bvh file that you want to broadcast")
    parser.add_argument('file_name', default="backfilp", help="the name of bvh file")
    parser.add_argument('base_frame', default="world", help="An existing frame in rviz on which the skeleton will be loaded")
    parser.add_argument('-n', '--name', default="BVHBroadcaster", help="Node name, default: BVHBroadcaster")
    parser.add_argument('-l', '--loop', action="store_true", help="Loop broadcasting")
    return parser.parse_args()

def main(args):
    print(args.name)
    # bvh_test = BVHBroadcaster(args.bvh_file, args.base_frame)
    print("Broadcasting bvh file {1} ({0}) on frame {2}".format(args.bvh_file, args.file_name, args.base_frame))
    if args.loop:
        print("Loop")
    else:
        print("Only once")
    # bvh_test.broadcast(loop = args.loop)



    # TODO: make data to quaternion
    # rotmat2quat(R)

    # TODO: lists
    save_file(args.file_name, lists)
    print("Broadcasting done and exit~")

def test():
    pass

if __name__ == '__main__':
    args = argsparser()
    # test(args)
    main(args)


'''
TODO: Data transfer
[
	duration of frame in seconds (1D),
	root position (3D),
	root rotation (4D),
	chest rotation (4D),
	neck rotation (4D),
	right hip rotation (4D),
	right knee rotation (1D),
	right ankle rotation (4D),
	right shoulder rotation (4D),
	right elbow rotation (1D),
	left hip rotation (4D),
	left knee rotation (1D),
	left ankle rotation (4D),
	left shoulder rotation (4D),
	left elbow rotation (1D)
]
'''