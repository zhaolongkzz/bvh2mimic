import math
import os
import string
import argparse
# import tf
import numpy as np
# from pyquaternion import Quaternion

import tf
import math
import numpy
import rospy
import string
import argparse

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
        # The current line number
        self.linenr = 0
        self.frames = 0

        self._root = None
        self._nodestack = []
        self._numchannels = 0

    def onHierarchy(self, root):
        pass

    def onMotion(self, frames, dt):
        pass

    def onFrame(self, values):
        pass

    def read(self):
        # self.fhandle = file(self.filename)
        self.fhandle = open(self.filename, 'r')

        self.readHierarchy()
        self.onHierarchy(self._root)
        self.readMotion()
    
    def readHierarchy(self):
        # FIRST read 'HIERARCHY' as the begining
        tok = self.token()
        if tok != 'HIERARCHY':
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

        self.frames = self.intToken()

        # Read the frame time
        tok = self.token()
        if tok!="Frame":
            raise SyntaxError("Syntax error in line %d: 'Frame Time:' expected, got '%s' instead"%(self.linenr, tok))
        tok = self.token()
        if tok!="Time:":
            raise SyntaxError("Syntax error in line %d: 'Frame Time:' expected, got 'Frame %s' instead"%(self.linenr, tok))

        dt = self.floatToken()

        self.onMotion(self.frames, dt)

        # Read the channel values
        for i in range(self.frames):
            s = self.readLine()
            a = s.split()
            if len(a)!=self._numchannels:
                raise SyntaxError("Syntax error in line %d: %d float values expected, got %d instead"%(self.linenr, self._numchannels, len(a)))
            values = list(map(lambda x: float(x), a))
            # map function => Python 2 return lists; Python 3 return iterators, so need a list
            self.onFrame(values)

    def readNode(self):
        name = self.token()
        self._nodestack[-1].name = name

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
        if self.tokenlist != []:
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
            self.linenr += 1
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


class BVHTransfer(BVHReader):
    def __init__(self, filename, root_frame):
        BVHReader.__init__(self, filename)
        self.br = tf.TransformBroadcaster()        # FIXME:
        # all the motions of BVH files
        self.all_motions = []
        # what the joint part I need for mimicking
        self.need_motions = []

        self.temp_quat = {}
        self.temp_chest = {}
        self.temp_neck = {}
        self.temp_lhip = {}
        self.temp_rhip = {}
        self.temp_larm = {}
        self.temp_rarm = {}

        # self.need_all_joints = [ "Hips", "LowerBack", "Spine", "Spine1", "Neck", "Neck1", 
        #                          "RightLeg", "RightFoot", "RightArm", "RightForeArm", "RightUpLeg",
        #                          "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftArm", "LeftForeArm"]
        self.chest = ["LowerBack", "Spine", "Spine1"]
        self.neck = ["Neck", "Neck1"]
        self.RHip = ["RHipJoint", "RightUpLeg"]
        self.RArm = ["RightShoulder", "RightArm"]
        self.LHip = ["LHipJoint", "LeftUpLeg"]
        self.LArm = ["LeftShoulder", "LeftArm"]

        self.need_joints = [ "Hips", "Spine1", "Neck1", 
                             "RightUpLeg", "RightLeg", "RightFoot", "RightArm", "RightForeArm",
                             "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftArm", "LeftForeArm",] 
        # self.need_joints = [ "Hips", "LowerBack", "Spine", "Spine1", "Neck", "Neck1", 
        #                      "RHipJoint", "RightUpLeg", "RightLeg", "RightShoulder", "RightArm",
        #                      "LHipJoint", "LeftUpLeg", "LeftLeg", "LeftShoulder", "LeftArm"]
        # self.need_joints = [ "Hips", "Spine1", "Neck1", 
        #                      "RHipJoint", "RightUpLeg", "RightLeg", "RightShoulder", "RightArm",
        #                      "LHipJoint", "LeftUpLeg", "LeftLeg", "LeftShoulder", "LeftArm"]              
        # self.revolute_joints_leg = ["RightLeg", "LeftLeg"]
        # self.revolute_joints_arm = ["RightForeArm", "LeftForeArm"]
        self.revolute_joints_leg = ["LeftLeg", "RightLeg"]
        self.revolute_joints_arm = ["RightForeArm", "LeftForeArm"]

        self.dt = 0.5
        self.num_motions = 1
        self.n = 10
        self.root_frame = root_frame

        self.counter = 0
        self.this_motion = None

        self.scaling_factor = 0.1

    def onHierarchy(self, root):
        self.scaling_factor = 0.1 / root.children[0].children[0].offset[0]
    
    def onMotion(self, frames, dt):
        self.dt = dt
        self.num_motions = frames

    def onFrame(self, values):
        self.all_motions.append(values)
            
    def rootJoint(self, root, parent_frame):
        if root.isEndSite():
            return
        
        num_channels = len(root.channels)
        flag_trans = 0
        flag_rot = 0

        mat_rot = np.array([ [1.,0.,0.,0.], 
                             [0.,1.,0.,0.], 
                             [0.,0.,1.,0.], 
                             [0.,0.,0.,1.] ])
        mat_rot_temp = np.array([ [1.,0.,0.,0.], 
                                [0.,1.,0.,0.], 
                                [0.,0.,1.,0.], 
                                [0.,0.,0.,1.] ])

        for channel in root.channels:
            keyval = self.this_motion[self.counter]
            if(channel == "Xposition"):
                flag_trans = True
                x = keyval
                self.need_motions.append(self.dt * self.n)
                self.need_motions.append(x * self.scaling_factor)
            elif(channel == "Yposition"):
                flag_trans = True
                y = keyval
                self.need_motions.append(y * self.scaling_factor)
            elif(channel == "Zposition"):
                flag_trans = True
                z = keyval
                self.need_motions.append(z * self.scaling_factor)
            elif(channel == "Xrotation"):
                flag_rot = True
                xrot = keyval
                # temp.append(xrot)
                theta = math.radians(xrot)
                c = math.cos(theta)
                s = math.sin(theta)
                mat_x_rot = np.array([ [1.,0.,0.,0.], 
                                       [0.,c,-s, 0.], 
                                       [0.,s, c, 0.], 
                                       [0.,0.,0.,1.] ])
                mat_rot = np.matmul(mat_rot, mat_x_rot)

            elif(channel == "Yrotation"):
                flag_rot = True
                yrot = keyval
                # temp.append(yrot)
                theta = math.radians(yrot)
                c = math.cos(theta)
                s = math.sin(theta)
                mat_y_rot = np.array([ [c, 0.,s, 0.],
                                       [0.,1.,0.,0.],
                                       [-s,0.,c, 0.],
                                       [0.,0.,0.,1.] ])
                mat_rot = np.matmul(mat_rot, mat_y_rot)

            elif(channel == "Zrotation"):
                flag_rot = True
                zrot = keyval
                # temp.append(zrot)
                theta = math.radians(zrot)
                c = math.cos(theta)
                s = math.sin(theta)
                mat_z_rot = np.array([ [c,-s, 0.,0.],
                                       [s, c, 0.,0.],
                                       [0.,0.,1.,0.],
                                       [0.,0.,0.,1.] ])
                mat_rot = np.matmul(mat_rot, mat_z_rot)
            else:
                return
            self.counter += 1
        
        if flag_trans:
            temp_trans = (self.scaling_factor * (x + root.offset[0]), 
                          self.scaling_factor * (y + root.offset[1]), 
                          self.scaling_factor * (z + root.offset[2]))
        else:
            temp_trans = (self.scaling_factor * (root.offset[0]), 
                          self.scaling_factor * (root.offset[1]), 
                          self.scaling_factor * (root.offset[2]))
        
        # compute the continuous several joints
        if root.name in self.chest:
            self.temp_chest[root.name] = mat_rot
            if root.name == "Spine1":
                for i in self.temp_chest:
                    mat_rot_temp = np.matmul(mat_rot_temp, self.temp_chest[i])
                mat_rot = mat_rot_temp
                parent_frame = "Hips"

        if root.name in self.neck:
            self.temp_neck[root.name] = mat_rot
            if root.name == "Neck1":
                for i in self.temp_neck:
                    mat_rot_temp = np.matmul(mat_rot_temp, self.temp_neck[i])
                mat_rot = mat_rot_temp
                parent_frame = "Spine1"

        elif root.name in self.LHip:
            self.temp_lhip[root.name] = mat_rot
            if root.name == "LeftUpLeg":
                for i in self.temp_lhip:
                    mat_rot_temp = np.matmul(mat_rot_temp, self.temp_lhip[i])
                mat_rot = mat_rot_temp
                parent_frame = "Hips"

        elif root.name in self.RHip:
            self.temp_rhip[root.name] = mat_rot
            if root.name == "RightUpLeg":
                for i in self.temp_rhip:
                    mat_rot_temp = np.matmul(mat_rot_temp, self.temp_rhip[i])
                mat_rot = mat_rot_temp
                parent_frame = "Hips"

        elif root.name in self.LArm:
            self.temp_larm[root.name] = mat_rot
            if root.name == "LeftArm":
                for i in self.temp_larm:
                    mat_rot_temp = np.matmul(mat_rot_temp, self.temp_larm[i])
                mat_rot = mat_rot_temp
                parent_frame = "Spine1"

        elif root.name in self.RArm:
            self.temp_rarm[root.name] = mat_rot
            if root.name == "RightArm":
                for i in self.temp_rarm:
                    mat_rot_temp = np.matmul(mat_rot_temp, self.temp_rarm[i])
                mat_rot = mat_rot_temp
                parent_frame = "Spine1"

        '''
        python: anim/KinTree.cpp:1574: static void cKinTree::LerpPoses(const Eigen::MatrixXd &,
         const Eigen::VectorXd &, const Eigen::VectorXd &, double,
          Eigen::VectorXd &): Assertion `std::abs(rot0.norm() - 1) < 0.000001' failed.
        
        if root.name in self.need_joints:
            if root.name in self.revolute_joints_leg:
                theta = math.radians(-xrot)
                temp_rot = np.array([theta])                          # knee and elbow is 1D
            elif root.name in self.revolute_joints_arm:
                theta = math.radians(xrot)
                temp_rot = np.array([theta])                          # knee and elbow is 1D
            else:
                temp_rot = self.quaternion_from_matrix(mat_rot)
        
                # if root.name in ["Hips", "Spine1", "Neck1", "RightLeg", "RightFoot", "RightArm", "RightForeArm", "RightUpLeg",
                #              "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftArm", "LeftForeArm"]:
                #     temp_rot = [1,0,0,0]
                # temp_rot = self.quaternion_from_matrix(mat_rot)        # all is 4D
            
            # save what joints we need, and put in dict in order
            mimic_dict = dict(enumerate(self.need_joints))
            self.temp_quat[list(mimic_dict.values()).index(root.name)] = temp_rot
        '''
        if root.name in self.need_joints:   
            temp_rot = self.quaternion_from_matrix(mat_rot)
            self.br.sendTransform(temp_trans, temp_rot, rospy.Time.now(), root.name, parent_frame)
        
        
        # FIXME:
        # temp_rot = tf.transformations.quaternion_from_matrix(mat_rot)
        # temp_rot = self.quaternion_from_matrix(mat_rot)
        
        # Cyclic traversal
        for each_child in root.children:
            self.rootJoint(each_child, root.name)
        return self.temp_quat

    def quaternion_from_matrix(self, matrix):
        """Return quaternion from rotation matrix.

        >>> R = rotation_matrix(0.123, (1, 2, 3))
        >>> q = quaternion_from_matrix(R)
        >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
        True
        """
        q = np.empty((4, ), dtype=np.float64)
        M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
        t = np.trace(M)
        if t > M[3, 3]:
            q[3] = t
            q[2] = M[1, 0] - M[0, 1]
            q[1] = M[0, 2] - M[2, 0]
            q[0] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
        q *= 0.5 / math.sqrt(t * M[3, 3])

        # return [q[3], q[2], q[1], q[0]]
        return q

    def rotmat2quat(self, R):
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
    
    def transfer(self, output_name):
        self.read()

        for ind in range(self.num_motions):
            self.counter = 0
            self.this_motion = self.all_motions[ind]

            temp_quat = self.rootJoint(self._root, self.root_frame)
            
            # transfer data to quaternion
            # quat = rotmat2quat(R)

            for i in range(len(self.need_joints)):
                # print(temp_quat[i])
                # print(self.need_motions)
                self.need_motions.extend(temp_quat[i])
        return self.need_motions, self.frames

    def broadcast(self, loop=False):
        self.read()
        rate = rospy.Rate(1/self.dt)

        while not rospy.is_shutdown():
            for ind in range(self.num_motions):
                self.counter = 0
                self.this_motion = self.all_motions[ind]

                self.rootJoint(self._root, self.root_frame)
                if rospy.is_shutdown():
                    break
                rate.sleep()
            if not loop:
                break
    
def save_file(name, loop, sample=False, lists=[[1,2,3],[2,3,4]]):
    """
    Save data to the default format of mimic
    """
    file_handle = open('./data/motions/humanoid3d_{}.txt'.format(name), mode='w')
    # file_handle = open('cmu_{}.txt'.format(name), mode='w')
    file_handle.write('{\n')
    file_handle.write('"Loop": "{0}",\n"Frames":\n[\n'.format(loop))
    # for i in range(frames):
    #     file_handle.write('[')
    #     for j in range(len(lists)):
    #         file_handle.write(str(lists[j]) + ',')
    #         if (j+1) % num_column == 0:
    #             file_handle.write(']' + ',\n')
    #             break
    # for j in range(len(lists)):    
    count = 1
    for i in lists:
        if count == len(lists):
            file_handle.write(str(list(i)) + '\n')
            break
        if sample and count % 10 == 0:
            file_handle.write(str(list(i)) + ',\n')
        count += 1

    file_handle.write(']\n}')
    file_handle.close()
    print('===> Save motion file to format of Mimic~\n')

def argsparser():
    parser = argparse.ArgumentParser("python BVHBroadcaster.py bvh_file base_frame --name --loop")
    parser.add_argument('bvh_file', default="./data/mocaps/22_13.bvh", help="A path to bvh file that you want to broadcast")
    parser.add_argument('base_frame', default="Hips", help="An existing frame in rviz on which the skeleton will be loaded")
    # parser.add_argument('-n', '--name', default="abackflip", help="The name of bvh file")
    # parser.add_argument('-l', '--loop', default="wrap", choices=["wrap", "none"], help="Loop broadcasting")
    parser.add_argument('-n', '--name', help="Node name, default: BVHBroadcaster", default="BVHBroadcaster")
    parser.add_argument('-l', '--loop', help="Loop broadcasting", action="store_true")
    return parser.parse_args()

def main(args):
    print("===> Name of output: humanoid3d_{}".format(args.name))
    bvh2mimic = BVHTransfer(args.bvh_file, args.base_frame)
    print("===> Transfering bvh file: humanoid3d_{1} ({0}) on frame {2}".format(args.bvh_file, args.name, args.base_frame))
    
    # Transfer CMU dataset to Mimic format
    need_motions, frames = bvh2mimic.transfer(args.name)
    num_column = int(len(need_motions) / frames)
    need_motions_temp = np.array(need_motions)
    need_motions = need_motions_temp.reshape(frames, num_column)
    
    # Save file
    if args.loop:
        print("===> Loop: wrap")
        sample = True
    else:
        print("===> Loop: none")
    save_file(args.name, args.loop, sample, need_motions)
    print("Transfering done and exit~\n")

    
def test(args):
    rospy.init_node(args.name)
    # file_name = "/home/mingfei/Documents/RobotManipulationProject/mocap/62/62_07.bvh"
    bvh_test = BVHTransfer(args.bvh_file, args.base_frame)
    rospy.loginfo("Broadcasting bvh file (%s) on frame %s"%(args.bvh_file, args.base_frame))
    if args.loop:
        rospy.loginfo("Loop")
    else: 
        rospy.loginfo("Only once")
    bvh_test.broadcast(loop=args.loop)
    rospy.loginfo("Broadcasting done")

if __name__ == '__main__':
    args = argsparser()
    test(args)
    # main(args)