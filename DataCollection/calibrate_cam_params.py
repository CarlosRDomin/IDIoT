"""
	Camera calibration based on OpenCV's cam calibration code. Takes (distorted) images from the selected camera,
	finds the calibration pattern on them (either a chessboard or an asymmetric circle grid),
	finds the optimal intrinsics of the camera model (camera calibration matrix and lens distortion coefficients),
	undistorts the images taken with the obtained intrinsic parameters and computes the reprojection error.

usage:
	calibrate.py [--debug <output path>] [--cell_size] [--is_chessboard] [<image mask>]

default values:
	--debug:			./output/
	--cell_size:		0.02	# Distance, in m, between 2 consecutive cell corners in the calibration pattern
	--is_chessboard:	False	# True if the calibration pattern is a chessboard. False, for asymmetric circle grid
	<image mask>:		None	# Could indicate the set of calibration images if already taken. Eg: ../data/calib*.jpg
								# But also, leave as "take_pics" to have the code help you collect calibration images
"""

from __future__ import print_function  # Python 2/3 compatibility

import numpy as np
import cv2
import sys
import os
import getopt
import vision_aux_functions as auxV
from glob import glob
from datetime import datetime


CALIB_PREFIX = "cam_calibration"
CALIB_FOLDER = "./{}/".format(CALIB_PREFIX)
CALIB_INPUT_PREFIX = "input"
CALIB_INPUT_FORMAT = "jpg"


def generate_calibration_filename(name):
	"""
	Produces a path inside the default calibration folder, so that all calibration-related files are saved together.
	:param name: Str containing the desired name for the file that will be saved inside the calibration-specific folder
	:return: Str with the path to the desired new file inside the calibration-specific folder
	"""
	return os.path.join(CALIB_FOLDER, "{}_{}".format(CALIB_PREFIX, name))


def ensure_folder_exists(folder):
	"""
	Makes sure that the given folder exists, by creating it if it didn't.
	:param folder: Str containing the path to the folder whose existence wants to be guaranteed
	"""
	if not os.path.isdir(folder):
		os.mkdir(folder)


def split_filename(filename):
	"""
	Splits a given filename into enclosing folder, file name and extension.
	:param filename: Path to a file whose components want to be split
	:return: Tuple containing the path of the enclosing folder, the file name, and the extension
	"""
	path, filename = os.path.split(filename)
	name, ext = os.path.splitext(filename)
	return path, name, ext


def take_calibration_images(cam_id=0):
	"""
	Captures calibration images by showing a live feed of the camera and saving the current image when Space is pressed.
	:param cam_settings: Path to the file to load the camera settings from (or None to use Spotter's default settings)
	"""
	v = cv2.VideoCapture(cam_id)

	ensure_folder_exists(CALIB_FOLDER)  # Make sure folder exists to prevent errors on file saving
	for file_name in glob(generate_calibration_filename("{}*".format(CALIB_INPUT_PREFIX))):
		os.remove(file_name)  # Remove any file from a previous calibration so they don't get mixed

	cnt_img = 0
	win_title = "Capturing calibration images - {} captured".format(cnt_img)
	cv2.namedWindow(win_title)
	cv2.moveWindow(win_title, 100,100)
	while True:
		ret, cam_frame = v.read()
		assert ret, "Couldn't grab a frame from cam {}".format(cam_id)
		cv2.imshow(win_title, cam_frame)

		key = cv2.waitKeyEx(1)
		if key == ord(' '):  # Save a new image when Space is pressed
			cnt_img += 1
			file_name = generate_calibration_filename("{}_{}.{}".format(CALIB_INPUT_PREFIX, cnt_img, CALIB_INPUT_FORMAT))
			cv2.setWindowTitle(win_title, "{} - {} captured".format(win_title.split(" - ")[0], cnt_img))
			cv2.imwrite(file_name, cam_frame)
			print("Captured new calibration image: {}".format(file_name))
		elif key >= 0:  # Exit if a key other than Space was pressed
			break

	cv2.destroyAllWindows()


if __name__ == '__main__':
	args, img_mask = getopt.getopt(sys.argv[1:], '', ['debug=', 'cell_size=', 'is_chessboard='])
	args = dict(args)
	args.setdefault('--debug', CALIB_FOLDER)
	args.setdefault('--cell_size', 0.02)
	args.setdefault('--is_chessboard', False)

	cam_id = 1  # Initialize calibration cam info
	if not img_mask or img_mask[0] == 'take_pics':  # If an image mask wasn't provided or user requests to capture calib images
		if img_mask:  # if img_mask[0] == 'take_pics', first take the calibration pictures
			take_calibration_images(cam_id)
		img_mask = generate_calibration_filename("{}*.{}".format(CALIB_INPUT_PREFIX, CALIB_INPUT_FORMAT))  # Obtain a list of all files that match the calibration filename prefix
	else:  # If an image mask was provided, use it
		img_mask = img_mask[0]

	# Get parameters from user-provided args
	img_names = glob(img_mask)
	debug_dir = args.get('--debug')
	ensure_folder_exists(debug_dir)  # Make sure folder exists to prevent errors on file saving
	cell_size = float(args.get('--cell_size'))
	is_chessboard = bool(args.get('--is_chessboard'))

	pattern_grid, pattern_points = auxV.get_calib_pattern_info(is_chessboard, cell_size)
	obj_points = []
	img_points = []
	h, w = 0, 0
	img_names_undistort = []
	for filename in img_names:  # For every file provided
		print('Processing {}... '.format(filename), end='')
		img = cv2.imread(filename, 0)  # Open the image in grayscale mode
		if img is None:
			print("Failed to load", filename)
			continue

		h, w = img.shape[:2]
		found, corners = auxV.find_calib_pattern(img, is_chessboard, pattern_grid)  # Find calibration pattern

		if debug_dir:
			vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
			cv2.drawChessboardCorners(vis, pattern_grid, corners, found)  # Draw the corners found in the image
			path, name, ext = split_filename(filename)
			outfile = os.path.join(debug_dir, '{}_chess.png'.format(name))
			cv2.imwrite(outfile, vis)  # And save it in the debug folder
			if found:  # If we were able to find the calibration pattern, add it to the list of valid calibration images
				img_names_undistort.append(outfile)

		if not found:  # Otherwise simply notify the user that the pattern could't be found
			print('Chessboard not found')
			continue

		img_points.append(corners.reshape(-1, 2))  # Store the image coordinates of each calibration corner/point found
		obj_points.append(pattern_points)

		print('ok')

	# Use OpenCV's methods to actually calibrate the camera and calculate camera distortion
	rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
	new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coefs, (w, h), 1)

	# Print output results and save them to a file
	print("\nRMS:", rms)
	print("Camera matrix:\n", camera_matrix)
	print("New camera matrix:\n", new_camera_matrix)
	print("Distortion coefficients: ", dist_coefs.ravel())
	print("ROI: ", roi)
	np.savez_compressed(generate_calibration_filename("output.npz"), rms=rms, camera_matrix=camera_matrix, dist_coefs=dist_coefs, rvecs=rvecs, tvecs=tvecs, cam_frame_size=(w, h), cam_id=cam_id, calib_at=datetime.now())

	# Undistort the images used for calibration with the new values found
	print('')
	for file_name in glob(generate_calibration_filename("*undistorted*")):
		os.remove(file_name)  # Remove any file from a previous calibration so they don't get mixed

	undist_maps = cv2.initUndistortRectifyMap(camera_matrix, dist_coefs, None, new_camera_matrix, (w, h), cv2.CV_16SC2)  # Generate undistortion mappings (will make undistort() much faster, since we are reusing the same mappings over and over)
	for img_found in img_names_undistort:
		img = cv2.imread(img_found)

		# h, w = img.shape[:2]
		t_start = datetime.now()
		dst = cv2.remap(img, undist_maps[0], undist_maps[1], cv2.INTER_LINEAR)
		# x, y, w, h = roi, dst = dst[y:y+h, x:x+w]  # Crop the image
		t_end = datetime.now()

		path, name, ext = split_filename(img_found)
		outfile = os.path.join(path, '{}_undistorted.png'.format(name))
		print('Undistorted image written to: {}; Took {:.2f}ms'.format(outfile, (t_end - t_start).total_seconds()*1000))
		cv2.imwrite(outfile, dst)

	total_error = 0
	for i in xrange(len(obj_points)):
		img_points2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coefs)
		error = cv2.norm(img_points[i], np.squeeze(img_points2), cv2.NORM_L2)/len(img_points2)
		total_error += error

	print("Total error: {}. Mean error: {}".format(total_error, total_error / len(obj_points)))
