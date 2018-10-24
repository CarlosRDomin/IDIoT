import cv2
import numpy as np
from config import AllParams
from process_helpers import ProcessImshowHelper
from paf_to_pose import paf_to_pose, plot_pose, people_to_pose
from scipy.optimize import linear_sum_assignment
from collections import deque
from enum import Enum
from datetime import datetime
import os
import h5py


class JointEnum(Enum):
    NOSE = 0
    NECK = 1
    RSHOULDER = 2
    RELBOW = 3
    RWRIST = 4
    LSHOULDER = 5
    LELBOW = 6
    LWRIST = 7
    RHIP = 8
    RKNEE = 9
    RANKLE = 10
    LHIP = 11
    LKNEE = 12
    LANKLE = 13
    REYE = 14
    LEYE = 15
    REAR = 16
    LEAR = 17


class MultiPersonTracker:
    """
     Associates the same person in two different frames to the same ID.
    """
    MIN_OVERLAP = 0.01

    def __init__(self, cost_old_person_not_tracked=2000, cost_new_person_starts_track=2000):
        self.COST_OLD_PERSON_NOT_TRACKED = cost_old_person_not_tracked
        self.COST_NEW_PERSON_STARTS_TRACK = cost_new_person_starts_track
        self.COST_DEFAULT = 2*(cost_old_person_not_tracked + cost_new_person_starts_track)
        self.last_joint_list = None
        self.last_person_to_joint_assoc = None
        self.last_bboxes = None

    def update(self, joint_list, person_to_joint_assoc):
        bboxes = find_all_people_bboxes(joint_list, person_to_joint_assoc)
        if self.last_person_to_joint_assoc is None:
            person_to_joint_assoc[:,-1] = range(person_to_joint_assoc.shape[0])  # Initialize IDs to 0...(N-1)
        else:
            M,N = len(self.last_person_to_joint_assoc), len(person_to_joint_assoc)
            assignment_cost = self.COST_DEFAULT*np.ones((M+N, M+N))
            assignment_cost[M:,N:] = 0
            np.fill_diagonal(assignment_cost[:M,N:], self.COST_OLD_PERSON_NOT_TRACKED)
            np.fill_diagonal(assignment_cost[M:,:N], self.COST_NEW_PERSON_STARTS_TRACK)

            for new_idx,new_person in enumerate(person_to_joint_assoc):
                # Compute the IoU overlap between this person and all others found in the previous frame
                overlap = np.maximum(compute_bbox_overlap(bboxes[new_idx], self.last_bboxes), self.MIN_OVERLAP)  # Avoid division by 0 by ensuring overlap >= MIN_OVERLAP (0.01)

                # Traverse every person found in last frame and compute matching score
                for old_idx,old_person in enumerate(self.last_person_to_joint_assoc):
                    common_joints = np.where(np.logical_and(old_person[0:-2]>=0, new_person[0:-2]>=0))[0]  # Figure out which joints were found both in the previous and in the current frame
                    score = self.COST_DEFAULT  # Default: very high score to ensure they're not matched
                    if len(common_joints) > 0:
                        old_joint_indices = old_person[common_joints].astype(int)
                        new_joint_indices = new_person[common_joints].astype(int)
                        old_joint_coords = self.last_joint_list[old_joint_indices, 0:2]
                        new_joint_coords = joint_list[new_joint_indices, 0:2]
                        score = np.mean(np.linalg.norm(new_joint_coords - old_joint_coords, axis=1)) / overlap[old_idx]  # Score: average distance weighted by 1/IoU
                    assignment_cost[old_idx, new_idx] = score

            old_tracks_idx, new_tracks_idx = linear_sum_assignment(assignment_cost)  # old_tracks_idx=range(M+N) and new_tracks_idx will indicate who each old_tracks_idx is matched to
            same_person_idx = np.where(new_tracks_idx[:M]<N)[0]  # Same person if the first M indices (each person found in the previous frame) are matched to a value smaller than N (matched to a person in the new frame)
            person_to_joint_assoc[new_tracks_idx[same_person_idx], -1] = self.last_person_to_joint_assoc[old_tracks_idx[same_person_idx], -1]  # Use same ID for same people
            new_person_idx = M + np.where(new_tracks_idx[M:]<N)[0]  # New person (new ID) if a person in the new frame (new_tracks_idx<N) is matched to one of the auxiliary rows with cost=self.COST_NEW_PERSON_STARTS_TRACK
            new_ID_start = max(self.last_person_to_joint_assoc[:,-1])+1 if M > 0 else 0
            person_to_joint_assoc[new_tracks_idx[new_person_idx], -1] = new_ID_start + np.arange(len(new_person_idx))

        self.last_joint_list = joint_list.copy()
        self.last_person_to_joint_assoc = person_to_joint_assoc.copy()
        self.last_bboxes = bboxes


class MultiWristTracker:
    """
     Helper class to plot a trail with the history positions of a given joint (e.g. wrist)
    """

    def __init__(self, limb_to_track=JointEnum.RWRIST.value, buff_size=40):
        self.limb_to_track = limb_to_track
        self.buff_size = buff_size
        self.trajectories = {'IDs': [], 'pts_deques': []}

    def update_trajectory(self, ind_trajectory, id, joint_list, person_to_joint_assoc):  # Finds the RWRIST of person with ID id, and appendsleft the coordinates to pts_deques[ind_trajectory]
        found = np.where(person_to_joint_assoc[:, -1] == id)[0]
        joint_idx = int(person_to_joint_assoc[found[0], self.limb_to_track]) if len(found) > 0 else -1
        self.trajectories['pts_deques'][ind_trajectory].appendleft(tuple(joint_list[joint_idx, 0:2].astype(int)) if joint_idx >= 0 else None)

    def update(self, joint_list, person_to_joint_assoc):
        for i, id in enumerate(self.trajectories['IDs']):
            self.update_trajectory(i, id, joint_list, person_to_joint_assoc)

        new_IDs = set(person_to_joint_assoc[:,-1]).difference(self.trajectories['IDs'])
        for id in new_IDs:
            self.trajectories['IDs'].append(id)
            self.trajectories['pts_deques'].append(deque(maxlen=self.buff_size))
            self.update_trajectory(len(self.trajectories['pts_deques'])-1, id, joint_list, person_to_joint_assoc)

    def plot(self, frame):
        color = (0, 0, 255)
        for pts in self.trajectories['pts_deques']:
            for i in range(1, len(pts)):
                if pts[i-1] is None or pts[i] is None:  # If either of the tracked points are None, ignore them
                    continue

                # otherwise, draw a connecting line
                thickness = int(np.sqrt(self.buff_size / float(i+1))*2.5)
                cv2.line(frame, pts[i-1], pts[i], color, thickness)


class FrameProcessor:
    """ Helper class to process a sequence of frames and display and/or save the processed results """
    FPS_ALPHA = 0.9

    def __init__(self, params=None):
        self.params = params if params is not None else AllParams()
        self.person_tracker = MultiPersonTracker()
        self.wrist_tracker = MultiWristTracker()
        self.pilot_bbox = None
        self.person_to_joint_assoc = None
        self.joint_list = None
        self.frame_num = 0
        self.fps = None
        self.filename_prefix = "cam_{}_{}".format(self.params.io.video_input, self.params.io.datetime_to_str(datetime.now())) if isinstance(self.params.io.video_input, int) else self.params.io.video_input.rsplit('.',1)[0]

        # Initialize h5 file (we'll save each processed frame's pose info in it)
        if self.params.io.pose_h5_file_handle is None:
            self.params.io.pose_h5_file_handle = h5py.File(self.filename_prefix + ".h5", 'a')  # Open file in append mode

            # First make sure we overwrite "params", "person_to_joint_assoc", "joint_list" (delete if already existed, to avoid problems/conflicts)
            for group in ("params", "person_to_joint_assoc", "joint_list"):
                if group in self.params.io.pose_h5_file_handle:
                    del self.params.io.pose_h5_file_handle[group]  # OVERWRITE (delete if already existed)

            # Save all params
            params_group = self.params.io.pose_h5_file_handle.create_group("params")
            for name, value in vars(self.params).iteritems():  # For each type of params (cam, pose, io)
                group = params_group.create_group(name)  # Create a subgroup
                for n, v in vars(value).iteritems():
                    if (name=="pose" and (n=="model" or n=="gpus")) or \
                       (name=="io" and n=="pose_h5_file_handle"):
                        continue  # Skip saving unnecessary params
                    group.attrs[n] = v if v is not None else "None"  # Save param value as attribute

            # Last, initialize person_to_joint_assoc and joint_list groups
            self.params.io.pose_h5_file_handle.create_group("person_to_joint_assoc")
            self.params.io.pose_h5_file_handle.create_group("joint_list")

        # Make sure "./Rendered/" folder exists (if save_render_as_video=False)
        imshow_helper_filename = self.params.io.TEMP_FILENAME
        if self.params.io.save_rendered_output and not self.params.io.save_render_as_video:  # All frames will go inside a folder (e.g. "./Rendered/")
            imshow_helper_filename = self.get_rendered_frame_filename()
            try:
                os.makedirs(os.path.dirname(self.get_rendered_frame_filename()))  # Make sure "./Rendered/" folder exists
            except:
                pass

        # Launch frame visualizer process
        if self.params.io.visualize_in_separate_process:  # Create a process to display the frames (+ skeleton) if requested
            self.imshow_helper = ProcessImshowHelper(imshow_helper_filename, 'Camera')
            print('Frame visualizer process spawned!')

        # Open the input (cam device or video filename)
        self.video = cv2.VideoCapture(self.params.io.video_input)
        self.img = self.img_resized_NN = self.img_out = None
        self.total_frames = int(self.video.get(cv2.CAP_PROP_FRAME_COUNT))  # If video, indicates the total number of frames (so we can log how much we've processed)
        fps = self.video.get(cv2.CAP_PROP_FPS)
        img_width = int(self.video.get(cv2.CAP_PROP_FRAME_WIDTH))
        img_height = int(self.video.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Initialize rendered video (if save_render_as_video=True)
        if self.params.io.save_rendered_output and self.params.io.save_render_as_video:
            self.video_out = cv2.VideoWriter(self.get_rendered_frame_filename(), cv2.VideoWriter_fourcc(*'avc1'), fps if fps > 0 else 25, (img_width, img_height))  # Note: avc1 is Apple's version of the MPEG4 part 10/H.264 standard apparently

        # Load the pose model
        load_pose_model(self.params)
        if not self.params.pose.use_openpose:
            self.resize_factor_for_nn = float(self.params.pose.net_resolution.split('x')[1]) / min(img_width, img_height)

        self.t_last_frame = datetime.now()

    def get_rendered_frame_filename(self):
        if self.params.io.save_rendered_output:
            if self.params.io.save_render_as_video:  # Save all rendered frames in a "*_rendered.mp4" video
                return "{}_rendered.mp4".format(self.filename_prefix)
            else:  # Save each individual frame inside a folder (e.g. "./Rendered/")
                filename_prefix = os.path.join(os.path.dirname(self.filename_prefix), self.params.io.RENDERED_FOLDER_NAME, os.path.basename(self.filename_prefix))  # Insert the "/Rendered/" folder in between
                return self.params.io.RENDERED_FRAME_FILENAME_FORMAT.format(filename_prefix, self.frame_num)  # Apply the format: ./Rendered/*_frame{N:05d}.jpg
        else:
            return self.params.io.TEMP_FILENAME

    def get_frame(self):
        # Read next frame
        ok, self.img = self.video.read()
        if not ok:
            print("ALL VIDEO FRAMES READ! :)" if self.total_frames > 0 and self.frame_num >= self.total_frames else "Error reading frame :(")
            return False

        # Mirror if needed
        if self.params.cam.bool_mirror:
            if self.params.pose.use_openpose:
                self.img = cv2.flip(self.img, 1)
            else:
                self.img = self.img[:, ::-1, :]

        # Resize if needed (for Ying's model)
        if not self.params.pose.use_openpose:
            self.img_resized_NN = cv2.resize(self.img, None, fx=self.resize_factor_for_nn, fy=self.resize_factor_for_nn)  # Downsize it to 248xW (where W is the corresponding width that preserves aspect ratio)

        # Increase frame counter
        self.frame_num += 1
        return True

    def run_pose_model(self):
        t1 = datetime.now()

        # Run the pose model
        if self.params.pose.use_openpose:
            model_output = self.params.pose.model.forward(self.img, self.params.pose.plot_skeletons)
            people_list, self.img_out = model_output if self.params.pose.plot_skeletons else (model_output, np.ascontiguousarray(self.img))
            self.joint_list, self.person_to_joint_assoc = people_to_pose(people_list)
        else:
            # Process the image at a lower resolution
            with torch.no_grad():
                img_input = Variable(torch.from_numpy(np.expand_dims(np.transpose(self.img_resized_NN.astype(np.float32), (2,0,1))/256-0.5, 0))).cuda(self.params.pose.gpus[0])  # Convert to Variable
                outputs, _ = self.params.pose.model(img_input)  # Run the image through the network

            # Modified by Long: run postprocess on GPU (transfered from caffe_rtpose)
            #  -> First run ./make.sh to compile it
            PAFs, heatmaps = outputs
            if self.params.pose.use_gpu_postprocess:
                people_list = rtpose_postprocess(heatmaps, PAFs, 8, num_part=18)[0]
                people_list[:, :, 0:2] /= self.resize_factor_for_nn
                self.joint_list, self.person_to_joint_assoc = people_to_pose(people_list)
                if self.params.pose.plot_skeletons:
                    _, self.img_out = plot_pose(self.img, self.joint_list, self.person_to_joint_assoc)
            else:
                _, self.img_out, self.joint_list, self.person_to_joint_assoc = paf_to_pose(self.img_resized_NN, tensor4d_to_numpy3d(heatmaps), tensor4d_to_numpy3d(PAFs), bool_plot=self.params.pose.plot_skeletons)
                if len(self.joint_list) > 0: self.joint_list[:, :2] /= self.resize_factor_for_nn

            if not self.params.pose.plot_skeletons:
                self.img_out = np.ascontiguousarray(self.img)

        if self.pilot_bbox is None:  # Initialize pilot if necessary
            self.pilot_bbox = find_largest_bbox(self.joint_list, self.person_to_joint_assoc)
        self.person_to_joint_assoc = find_same_person(self.pilot_bbox, self.joint_list, self.person_to_joint_assoc)  # Reorder person_to_joint_assoc based on overlap with pilot_bbox
        self.person_tracker.update(self.joint_list, self.person_to_joint_assoc)  # Assign same ID to people that appeared on last frame
        self.wrist_tracker.update(self.joint_list, self.person_to_joint_assoc)  # Track each person's wrist
        self.wrist_tracker.plot(self.img_out)  # Plot a trail of points with the history of wrist positions
        self.pilot_bbox = draw_bbox(self.img_out, self.joint_list, self.person_to_joint_assoc, True)  # And add a bounding-box around the main target

        t2 = datetime.now()
        if False:
            print("MODEL RUNS IN: {:6.2f}ms".format(1000*(t2-t1).total_seconds()))

    def save_frame_results(self):
        # Save pose results to the file (if needed)
        if self.params.io.pose_h5_file_handle is not None:
            for pose_field in ("person_to_joint_assoc", "joint_list"):
                field_h5_group = self.params.io.pose_h5_file_handle[pose_field]
                field_h5_group.create_dataset(self.params.io.FRAME_FILENAME_FORMAT.format(self.frame_num), data=getattr(self, pose_field))

        # Helper function to save img_out as a temp jpg for visualization in the other process (if requested through params.io)
        def save_temp_frame_if_needed_for_visualization():
            if self.params.io.visualize_in_separate_process:
                cv2.imwrite(self.params.io.TEMP_FILENAME, self.img_out, (cv2.IMWRITE_JPEG_QUALITY, 25))  # Only used for visualization, so quality 25 is enough

        # Save img_out to file
        if self.params.io.save_rendered_output:
            if self.params.io.save_render_as_video:
                self.video_out.write(self.img_out)
                save_temp_frame_if_needed_for_visualization()
            else:
                rendered_frame_filename = self.get_rendered_frame_filename()
                cv2.imwrite(rendered_frame_filename, self.img_out, (cv2.IMWRITE_JPEG_QUALITY, 80))
                if self.params.io.visualize_in_separate_process:  # If visualizing in a separate process, no need to re-save the image in low quality, reuse the one we just saved
                    self.imshow_helper.set_filename(rendered_frame_filename)  # Update filename to visualize the latest frame we just saved
        else:
            save_temp_frame_if_needed_for_visualization()  # Only save temp jpg if visualizing in a separate process

        # Visualize it in this process if requested
        if not self.params.io.visualize_in_separate_process:
            cv2.imshow("Camera", self.img_out)

    def process_kb(self):
        if self.params.io.visualize_in_separate_process:
            keys = self.imshow_helper.get_keys_pressed()
        else:
            key = cv2.waitKeyEx(1)
            keys = [key] if key >=0 else []

        should_exit = False
        for key in keys:
            print('KEY PRESSED: {} ({})'.format(key, chr(key) if key < 256 else 'special ch'))
            if key == ord('q'):
                should_exit = True

        return not should_exit

    def update_fps(self):
        fps_curr = 1 / (datetime.now() - self.t_last_frame).total_seconds()
        self.fps = fps_curr if self.fps is None else self.FPS_ALPHA*self.fps + (1-self.FPS_ALPHA)*fps_curr
        progress_info = "/{} ({:5.2f}%)".format(self.total_frames, 100.0*self.frame_num/self.total_frames) if self.total_frames > 0 else ""
        print("Curr frame: {:5d}{}; FPS: smoothed {:5.2f}\tinstantaneous {:5.2f}".format(self.frame_num, progress_info, self.fps, fps_curr))
        self.t_last_frame = datetime.now()

    def terminate(self):
        self.video.release()  # Close cam/video input
        cv2.destroyAllWindows()  # Close all cv2 windows
        if self.params.io.pose_h5_file_handle is not None:
            self.params.io.pose_h5_file_handle.close()  # Close pose file
        if self.params.io.visualize_in_separate_process and not (self.params.io.save_rendered_output and not self.params.io.save_render_as_video):
            os.remove(self.params.io.TEMP_FILENAME)  # Delete temp file (used for visualization)

    def run_demo(self):
        while True:
            if not self.get_frame():
                break
            self.run_pose_model()
            self.save_frame_results()
            if not self.process_kb():
                break
            self.update_fps()
        print('EXITING!')
        self.terminate()


def load_pose_model(params=None):
    if params is None:
        params = AllParams()

    if params.pose.use_openpose:
        from openpose import OpenPose

        params.pose.model = OpenPose(vars(params.pose))
    else:
        import torch
        import torch.nn as nn
        from torch.autograd import Variable
        from network.Ying_model import get_ying_model
        if params.pose.use_gpu_postprocess:
            from cpm.cpm_layer import rtpose_postprocess

        model = get_ying_model(stages=5, have_bn=False, have_bias=True)
        model.load_state_dict(torch.load('./models/dilated3_5stage_merged.pth'))
        model = nn.DataParallel(model, device_ids=params.pose.gpus)
        params.pose.model = model.cuda(params.pose.gpus[0])
        params.pose.model.eval()

    return params


def compute_bbox_overlap(reference_bbox, bboxes):
    """ Compute the IoU overlap between a given bbox and a set of bboxes """
    reference_bbox = np.reshape(reference_bbox, (-1, 4))  # Ensure they are 2d arrays and not 1d (in case there's only one person found)
    bboxes = np.reshape(bboxes, (-1, 4))  # Make sure bboxes is an np.array so we can compute all overlaps at once

    minimum_coords = np.minimum(reference_bbox, bboxes)
    maximum_coords = np.maximum(reference_bbox, bboxes)
    union_bboxes        = np.hstack((minimum_coords[:,:2], maximum_coords[:,2:]))
    intersection_bboxes = np.hstack((maximum_coords[:,:2], minimum_coords[:,2:]))

    intersection_width  = np.maximum(0, intersection_bboxes[:,2]-intersection_bboxes[:,0]+1)
    intersection_height = np.maximum(0, intersection_bboxes[:,3]-intersection_bboxes[:,1]+1)
    union_width  = np.maximum(0, union_bboxes[:,2]-union_bboxes[:,0]+1)
    union_height = np.maximum(0, union_bboxes[:,3]-union_bboxes[:,1]+1)

    overlap = (intersection_width*intersection_height) / (union_width*union_height)
    return overlap


def find_same_person(reference_bbox, joint_list, person_to_joint_assoc):
    """ Sort the rows in person_to_joint_assoc based on bounding-box overlap with the reference_bbox (highest overlap first) """
    bboxes = find_all_people_bboxes(joint_list, person_to_joint_assoc)
    if len(bboxes) == 0:  # Nothing to do, there's no people in the frame!
        return person_to_joint_assoc

    overlap = compute_bbox_overlap(reference_bbox, bboxes)

    def custom_sort_based_on_overlap(i, j):  # Both i and j are tuples, where [0] is an overlap percentage, and [1] is their person_info
        if i[0] < j[0]:
            return -1
        elif i[0] > j[0]:
            return 1
        else:  # Exactly the same overlap (eg: 0% overlap)
            return cmp(i[1][-2], j[1][-2])  # Return whoever has higher joint_score (person[-2])

    return np.asarray([x for (y, x) in sorted(zip(overlap, person_to_joint_assoc), cmp=custom_sort_based_on_overlap, reverse=True)])  # Sort person_to_joint_assoc based on overlap (higher overlap = lower index)


def find_largest_bbox(joint_list, person_to_joint_assoc):
    """ Return the bounding-box among all people found in the image with largest area """
    max_bbox_area = -1
    max_bbox = None

    for person_ind, person in enumerate(person_to_joint_assoc):
        bbox = find_person_bbox(person, joint_list)
        current_area = (bbox[2]-bbox[0]) * (bbox[3]-bbox[1])
        if current_area >= max_bbox_area:
            max_bbox_area = current_area
            max_bbox = bbox

    return max_bbox


def find_person_bbox(person, joint_list):
    """ Return the bounding-box corresponding to the given person (specified by a row from person_to_joint_assoc) """
    person_joints_found = np.where(person[0:-2] >= 0)[0]
    joint_indices = person[person_joints_found].astype(int)
    joint_coords = joint_list[joint_indices, 0:2]
    xy_min = np.min(joint_coords, axis=0)
    xy_max = np.max(joint_coords, axis=0)
    return np.hstack((xy_min, xy_max))


def find_all_people_bboxes(joint_list, person_to_joint_assoc):
    """ Return a list of bounding boxes, one for each person found in the image """
    bboxes = []
    for person_ind, person in enumerate(person_to_joint_assoc):
        bboxes.append(find_person_bbox(person, joint_list))
    return bboxes


def draw_bbox(canvas, joint_list, person_to_joint_assoc, bool_plot=True, bool_plot_pilot_only=False, id=None):
    """ Draw bounding boxes around people found in the image (or only the main target if bool_plot_pilot_only=True) """
    if id is None and len(person_to_joint_assoc) > 0:
        id = person_to_joint_assoc[0,-1]
    COLOR_PILOT     = (0, 255, 0)
    COLOR_NOT_PILOT = (0, 0, 255)
    THICKNESS_PILOT     = 2
    THICKNESS_NOT_PILOT = 1

    pilot_bbox = None
    for person_ind, person in enumerate(person_to_joint_assoc):
        bbox = find_person_bbox(person, joint_list)
        if person[-1] == id:
            pilot_bbox = bbox
        if bool_plot and (person[-1] == id or not bool_plot_pilot_only):
            xy_min = np.round(bbox[:2]).astype(int)
            xy_max = np.round(bbox[2:]).astype(int)
            color = COLOR_PILOT if person[-1] == id else COLOR_NOT_PILOT
            thickness = THICKNESS_PILOT if person[-1] == id else THICKNESS_NOT_PILOT
            cv2.rectangle(canvas, tuple(xy_min), tuple(xy_max), color, thickness=thickness)
            cv2.putText(canvas, str((person[-1] if True else person_ind)+1), tuple(xy_min+[0, 25]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 1, cv2.LINE_AA)
            if False and person[-1] == id:
                cv2.putText(canvas, 'PILOT', tuple(xy_min-[0,15]), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 1, cv2.LINE_AA)

    return pilot_bbox


def tensor4d_to_numpy3d(in_tensor):
    return in_tensor.squeeze().permute(1,2,0).cpu().detach().numpy()


