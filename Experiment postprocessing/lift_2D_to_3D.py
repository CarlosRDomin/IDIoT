import torch
import torch.nn as nn
import numpy as np
import matplotlib.pyplot as plt


class LinearLayer(nn.Module):
    def __init__(self, linear_size, p_dropout=0.5):
        super(LinearLayer, self).__init__()
        self.l_size = linear_size

        self.relu = nn.ReLU(inplace=True)
        self.dropout = nn.Dropout(p_dropout)

        self.w1 = nn.Linear(self.l_size, self.l_size)
        self.batch_norm1 = nn.BatchNorm1d(self.l_size)

        self.w2 = nn.Linear(self.l_size, self.l_size)
        self.batch_norm2 = nn.BatchNorm1d(self.l_size)

    def forward(self, x):
        y = self.w1(x)
        y = self.batch_norm1(y)
        y = self.relu(y)
        y = self.dropout(y)

        y = self.w2(y)
        y = self.batch_norm2(y)
        y = self.relu(y)
        y = self.dropout(y)

        out = x + y

        return out


class Model2Dto3Dpose(nn.Module):
    def __init__(self,
                 linear_size=1024,
                 num_stage=2,
                 p_dropout=0.5):
        super(Model2Dto3Dpose, self).__init__()

        self.linear_size = linear_size
        self.p_dropout = p_dropout
        self.num_stage = num_stage

        # 2d joints
        self.input_size =  16 * 2
        # 3d joints
        self.output_size = 16 * 3

        # process input to linear size
        self.w1 = nn.Linear(self.input_size, self.linear_size)
        self.batch_norm1 = nn.BatchNorm1d(self.linear_size)

        self.linear_stages = []
        for l in range(num_stage):
            self.linear_stages.append(LinearLayer(self.linear_size, self.p_dropout))
        self.linear_stages = nn.ModuleList(self.linear_stages)

        # post processing
        self.w2 = nn.Linear(self.linear_size, self.output_size)

        self.relu = nn.ReLU(inplace=True)
        self.dropout = nn.Dropout(self.p_dropout)

    def forward(self, x):
        # pre-processing
        y = self.w1(x)
        y = self.batch_norm1(y)
        y = self.relu(y)
        y = self.dropout(y)

        # linear layers
        for i in range(self.num_stage):
            y = self.linear_stages[i](y)

        y = self.w2(y)

        return y


class Helper2Dto3Dpose:
    # After un-normalization, only following dimensions are used
    DIMS_TO_USE_AFTER_UNNORM = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 18, 19, 20, 21, 22, 23, 24, 25, 26, 36, 37, 38, 39, 40, 41, 42, 43, 44, 51, 52, 53, 54, 55, 56, 57, 58, 59, 75, 76, 77, 78, 79, 80, 81, 82, 83]  # No head

    def __init__(self, model=None, stat_3d=None):
        self.stat_3d = stat_3d if stat_3d is not None else torch.load('2D to 3D/stat_3d.pth.tar', map_location='cpu')
        # Preprocess stat_3d so that mean and std are row vectors
        # D = self.stat_3d['mean'].shape[0]  # 96
        # self.stat_3d['mean'] = self.stat_3d['mean'].reshape((1, D))
        # self.stat_3d['std'] = self.stat_3d['std'].reshape((1, D))

        # Load model if needed
        if model is not None:
            self.model = model
        else:
            self.model = Model2Dto3Dpose()
            ckpt = torch.load('2D to 3D/gt_ckpt_best.pth.tar', map_location='cpu')
            self.model.load_state_dict(ckpt['state_dict'])
            self.model.eval()

        # Initialize 2D and 3D data arrays
        self.data_2D = []
        self.data_3D = []


    @staticmethod
    def parse_openpose_format(pose_2D):  # pose_2D is num_people x 3*num_joints, where each row contains x1,y1,confidence1,x2,y2...
        unordered_joints = np.zeros((pose_2D.shape[0], 16, 2))  # Initialize joint list: num_people x 16 (joints) x 2 (x,y coords)
        # Take the (x,y) of the first 15 joints from openpose, flipping the y axis because origin is expected to be bottom-left
        unordered_joints[:,:-1,0] =  pose_2D[:, 0:3*15:3]  # x
        unordered_joints[:,:-1,1] = -pose_2D[:, 1:3*15:3]  # y
        # 16th joint is spine: middle point between joints 1 (neck) and 8 (r_hip)
        unordered_joints[:,-1,0] = (pose_2D[:, 1*3]+pose_2D[:, 8*3])/2
        unordered_joints[:,-1,1] = -(pose_2D[:, 1*3+1]+pose_2D[:, 8*3+1])/2

        # Reorder joints to match the expected format
        joint_reorder_list = [8, 12, 13, 14, 9, 10, 11, 15, 1, 0, 2, 3, 4, 5, 6, 7]
        ordered_joints = unordered_joints[:, joint_reorder_list, :]

        # Normalize joints (move hip to (0,0) and divide by empirical factor)
        ordered_joints = ordered_joints - ordered_joints[:, 0, :]  # Translate the skeleton such that each hip is at (0,0)
        ordered_joints = ordered_joints / 100  # Normalize. 100 is picked after looking at the data from openpose
        return ordered_joints.reshape((ordered_joints.shape[0], -1))  # Convert to num_people x 32 where each row contains x1,y1,x2,y2,x3...

    @staticmethod
    def to_openpose_format(pose_3D, num_joints):
        openpose_3D = np.zeros((pose_3D.shape[0], num_joints, 3))  # Initialize joint list: num_people x num_joints (typically 25) x 3 (x,y,z coords)

        # Reverse the reordering applied in parse_openpose_format
        joint_reorder_list = [9, 8, 10, 11, 12, 13, 14, 15, 0, 4, 5, 6, 1, 2, 3]  # Ignore the '16th' element of unordered_joints because that's not an openpose joint, just a "virtual" joint required by the lift3D model
        openpose_3D[:, :15, :] = pose_3D.reshape((-1,16,3))[:, joint_reorder_list, :]
        return openpose_3D.reshape((-1,num_joints*3))


    def unnormalize_data(self, normalized_data):
        T = normalized_data.shape[0]  # Batch size

        # Fill in orig_data with the right format
        orig_data = np.zeros((T, len(self.stat_3d['mean'])), dtype=np.float32)
        orig_data[:, self.stat_3d['dim_use']] = normalized_data

        # Unnormalize: multiply times stdev and add the mean
        stdMat = np.repeat(self.stat_3d['std'], T, axis=0)
        meanMat = np.repeat(self.stat_3d['mean'], T, axis=0)
        orig_data = np.multiply(orig_data, stdMat) + meanMat

        return orig_data[:, self.DIMS_TO_USE_AFTER_UNNORM]  # Filter the result to only return the 'interesting' dimensions

    def process_2D(self, pose_2D, is_openpose=True, append_data=False):
        # Prepare input so it can be understood by the model
        if is_openpose:
            num_joints = pose_2D.shape[1]/3
            pose_2D = self.parse_openpose_format(pose_2D)
        # pose_2D = pose_2D.reshape((1,-1))  # Ensure row vector
        self.data_2D = pose_2D if not append_data else np.vstack((self.data_2D, pose_2D))  # Save or append in data_2D

        # Run the input through the model and save the output
        out_3D = self.model(torch.from_numpy(pose_2D).float()).detach().numpy()
        out_3D_unnorm = self.unnormalize_data(out_3D)
        if is_openpose:
            out_3D_unnorm = self.to_openpose_format(out_3D_unnorm, num_joints)  # Convert back to num_people x num_joints*3 (x1,y1,z1, x2,y2,z2, ...)
        self.data_3D = out_3D_unnorm if not append_data else np.vstack((self.data_3D, out_3D_unnorm))  # Save or append in data_3D

    @staticmethod
    def render_2D_pose(pose_2D, ax):
        lcolor="#3498db"
        rcolor="#e74c3c"
        vals = np.reshape(pose_2D, (16, -1))
        I  = np.array([1,2,3,1,5,6,1,8, 9, 9,11,12, 9,14,15])-1 # start points
        J  = np.array([2,3,4,5,6,7,8,9,10,11,12,13,14,15,16])-1 # end points
        LR = np.array([1,1,1,0,0,0,0, 0, 0, 0, 0, 0, 1, 1, 1], dtype=bool)

        # Make connection matrix
        for i in np.arange( len(I) ):
            x, y = [np.array( [vals[I[i], j], vals[J[i], j]] ) for j in range(2)]
            ax.plot(x, y, lw=2, c=lcolor if LR[i] else rcolor)

        for j in np.arange( 16 ):
            x, y = vals[j, 0], vals[j, 1]
            ax.annotate( str(j), (x, y), size=7, weight='bold')

    @staticmethod
    def render_3D_pose(pose_3D, ax):
        lcolor="#3498db"
        rcolor="#e74c3c"
        vals = np.reshape(pose_3D, (16, -1))
        I  = np.array([1,2,3,1,5,6,1,8, 9, 9,11,12, 9,14,15])-1 # start points
        J  = np.array([2,3,4,5,6,7,8,9,10,11,12,13,14,15,16])-1 # end points
        LR = np.array([1,1,1,0,0,0,0, 0, 0, 0, 0, 0, 1, 1, 1], dtype=bool)

        # Make connection matrix
        for i in np.arange( len(I) ):
            x, y, z = [np.array( [vals[I[i], j], vals[J[i], j]] ) for j in range(3)]
            ax.plot(x, y, z, lw=2, c=lcolor if LR[i] else rcolor)
        for j in np.arange( 16 ):
            x, y, z = vals[j, 0], vals[j, 1], vals[j, 2]
            ax.text(x, y, z, str(j))
