import numpy as np


class PoseParams:
    """ Structure that holds the configuration parameters for the human pose model """

    def __init__(self, gpus=None, plot_skeletons=True, use_gpu_postprocess=False, use_openpose=True, output_resolution="-1x-1", model_pose="BODY_25", default_model_folder="openpose/models/"):
        self.model = None  # Variable that will hold the pose model on which we call .forward(img)
        self.gpus = gpus if gpus is not None else [0]  # GPU(s) to use for running the NN model
        self.use_gpu_postprocess = use_gpu_postprocess  # (Ignored if use_openpose=True) Determines whether the postprocessing&plotting of pose is ran on GPU (True) or CPU (False)
        self.plot_skeletons = plot_skeletons  # Whether or not to draw skeletons on top of people
        self.use_openpose = use_openpose  # True for OpenPose CPM's model, False for Ying's model

        # Params needed by OpenPose:
        self.net_resolution = "-1x{}".format(368 if use_openpose else 248)
        self.output_resolution = output_resolution
        self.model_pose = model_pose
        self.logging_level = 3
        self.alpha_pose = 0.6
        self.scale_gap = 0.3
        self.scale_number = 1
        self.render_threshold = 0.05
        self.num_gpu_start = self.gpus[0]
        self.disable_blending = False
        self.default_model_folder = default_model_folder

    def update_img_size(self, img_size):
        self.net_resolution = "-1x{}".format(368 if img_size is None else img_size[1]/2)  # 'x'.join(str(x) for x in img_size)


class CamParams:
    """ Structure that holds the configuration parameters for the camera-related settings """

    def __init__(self, bool_mirror=False, focal_length_in_px=1250.0, camera_matrix=None, dist_coefs=None):
        self.bool_mirror = bool_mirror  # Whether or not to mirror the input image
        self.focal_length_in_px = focal_length_in_px
        self.camera_matrix = camera_matrix if camera_matrix is not None else np.matrix([[focal_length_in_px, 0, 1280./2], [0, focal_length_in_px, 720./2], [0, 0, 1]])
        self.dist_coefs = dist_coefs


class InOutParams:
    """ Structure that holds the configuration parameters for IO-related settings """
    FRAME_FILENAME_FORMAT = "frame{:05d}"
    RENDERED_FRAME_FILENAME_FORMAT = "{}_" + FRAME_FILENAME_FORMAT + ".jpg"
    RENDERED_FOLDER_NAME = "Rendered"
    TEMP_FILENAME = "frame.jpg"

    def __init__(self, video_input=0, pose_h5_file_handle=None, save_rendered_output=True, save_render_as_video=True, visualize_in_separate_process=True):
        self.video_input = video_input  # Camera id or video filename to pass to cv2.VideoCapture
        self.pose_h5_file_handle = pose_h5_file_handle  # h5 file handle to which to add a dataset with each frame's pose info (None skips saving)
        self.save_rendered_output = save_rendered_output  # Whether or not to save each frame's image+pose render
        self.save_render_as_video = save_render_as_video  # In case save_rendered_output is True, this determines whether to save all frames in a video (True) or as individual jpg's (False)
        self.visualize_in_separate_process = visualize_in_separate_process  # Whether or not to display each processed frame in a separate process (better performance)

    @staticmethod
    def datetime_to_str(t):
        return str(t)[:-7].replace(':', '-')


class AllParams:
    """ Structure that holds the configuration parameters for the overall project """

    def __init__(self, video_input=0, use_openpose=True, pose=None, cam=None, io=None):
        self.pose = pose if pose is not None else PoseParams(use_openpose=use_openpose)  # Pose params
        self.cam = cam if cam is not None else CamParams()  # Camera params
        self.io = io if io is not None else InOutParams(video_input=video_input)  # IO params
