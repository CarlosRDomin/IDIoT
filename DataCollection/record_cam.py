"""
Basic script to save a video from a USB camera
"""

import cv2
import numpy as np
import os
import h5py
from datetime import datetime
from log_helper import logger


def find_avail_cams():
    video_capture = cv2.VideoCapture()
    for i in range(1501):
        if video_capture.open(i):
            logger.info("\tCAMERA {} OPENED!".format(i))
            for j in range(3):  # Read a couple frames, sometimes cameras return a full-green frame on the first read()
                ret, frame = video_capture.read()
            video_capture.release()  # Close the camera
            if ret:
                cv2.imwrite("cam_{}.jpg".format(i), frame)
            else:
                logger.error("Ooops, something went wrong accessing the frame! :S")
        else:
            logger.debug("Nothing on {}...".format(i))


def record_cam(cam, out_folder='data', t_start=None, save_as_video=True):
    if t_start is None: t_start = datetime.now()  # Initialize t_start to current time if t_start wasn't specified
    if isinstance(t_start, datetime): t_start = str(t_start)[:-7].replace(':', '-')  # Convert to str
    out_folder = os.path.join(out_folder, t_start)
    os.makedirs(out_folder, exist_ok=True)  # Ensure folder exists

    # Open the camera
    video_capture = cv2.VideoCapture(cam)
    fps = video_capture.get(cv2.CAP_PROP_FPS)
    ret, frame = video_capture.read()  # Read a frame, sometimes first read() returns an "invalid" image
    if not ret:
        logger.critical("Oops, can't access cam {}, exiting! :(".format(cam))
        return
    frame_dimensions = frame.shape[1::-1]  # width, height

    # Create a video file if save_as_video
    if save_as_video:
        video_filename = os.path.join(out_folder, "cam_{}_{}.mp4".format(cam, t_start))
        video_writer = cv2.VideoWriter(video_filename, cv2.VideoWriter_fourcc(*'avc1'), fps if fps > 0 else 25, frame_dimensions)  # Note: avc1 is Apple's version of the MPEG4 part 10/H.264 standard apparently
        # logger.info("Bye")
        # return
    logger.notice("Starting cam '{}' recording! Saving {}".format(cam, "as {}".format(video_filename) if save_as_video else "at {}".format(out_folder)))

    t_frames = []
    try:
        while True:
            ret, frame = video_capture.read()
            t_frames.append(datetime.now())
            if len(t_frames) > 1: logger.debug("Wrote frame {} with delta={:.3f}ms (saving took {:.3f}ms)".format(len(t_frames), 1000*(t_frames[-1]-t_frames[-2]).total_seconds(), 1000*(t_save-t_frames[-2]).total_seconds()))

            if not ret:
                logger.critical("Unknown error capturing a frame from cam {}!")
            elif save_as_video:
                video_writer.write(frame)
            else:  # Save as still image
                cv2.imwrite(os.path.join(out_folder, "cam_{}_f{:05d}_{}.jpg".format(cam, len(t_frames), t_frames[-1].strftime("%H-%M-%S-%f"))), frame)
            t_save = datetime.now()
    except KeyboardInterrupt:
        logger.notice("Stopping cam recording!")
    finally:
        video_capture.release()
        info_filename = os.path.join(out_folder, "cam_{}_{}.h5".format(cam, t_start))
        with h5py.File(info_filename) as hf:
            hf.attrs["t_start"] = t_start
            hf.attrs["fps"] = fps
            hf.attrs["width"] = frame_dimensions[0]
            hf.attrs["height"] = frame_dimensions[1]
            hf.create_dataset("t_frames", data=np.array([t.timestamp() for t in t_frames]))

    logger.success("Goodbye from cam {} recording!".format(cam))


if __name__ == '__main__':
    record_cam(1)
