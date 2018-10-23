"""
	Helper functions for vision stuff: coordinate transforms (distorted, undistorted, camera and world frames of reference)
	and calibration-related methods (find chessboard pattern, find focal length, compute extrinsic transformation...)
"""

import numpy as np
import cv2


def get_cam_matrix_and_dist_coefs(camera_matrix_or_calib_file, dist_coefs=None):
	"""
	Helper function that returns camera_matrix and dist_coefs, either loading them from a file
	(if camera_matrix_or_calib_file is str) or passing them through otherwise.
	:param camera_matrix_or_calib_file: If str, indicates the name of the calibration file to load camera_matrix and dist_coefs from.
	If np.matrix, it is the camera_matrix itself and camera_matrix and dist_coefs will be returned unmodified.
	:param dist_coefs: If camera_matrix_or_calib_file is str, it has no effect (leave as None);
	Otherwise, it should be a 1x5 np.array containing [k1 k2 p1 p2 k3] lens distorsion coefficients.
	:return: camera_matrix (intrinsics of the camera model) and dist_coefs (lens distorsion coefficients).
	"""
	if isinstance(camera_matrix_or_calib_file, str):  # If camera_matrix_or_calib_file is str, it indicates the calibration file
		with np.load(camera_matrix_or_calib_file) as data:  # Load required params from *.npz file
			dist_coefs = data["dist_coefs"][0]
			camera_matrix = data["camera_matrix"]
	else:  # If camera_matrix_or_calib_file is not str, then it's the camera matrix itself. Do nothing.
		camera_matrix = camera_matrix_or_calib_file

	return camera_matrix, dist_coefs

def dist_to_undist_img_coords(distorted_pt, camera_matrix_or_calib_file, dist_coefs=None):
	"""
	Given the *distorted* pixel coordinates (u_dist, v_dist), retrieve the pixel location on *undistorted* image
	coordinates (u_undist, v_undist). It is the reverse operation of undist_to_dist_img_coords, and simply calls cv2.undistortPoints.
	:param distorted_pt: Nx2 np.array with the location of point(s) in *distorted* image coordinates (u_dist, v_dist).
	:param camera_matrix_or_calib_file: If str, indicates the name of the calibration file to load camera_matrix and dist_coefs from.
	Otherwise, should be a 3x3 np.matrix containing the intrisics of the camera model.
	:param dist_coefs: 1x5 np.array containing the distortion coefficients of the lens [k1, k2, p1, p2, k3]
	(ignored if camera_matrix_or_calib_file is str).
	:return: Nx2 np.array with the location of distorted_pt in *undistorted* image coordinates (u_undist, v_undist).
	"""
	camera_matrix, dist_coefs = get_cam_matrix_and_dist_coefs(camera_matrix_or_calib_file, dist_coefs)
	distorted_pt = np.matrix(distorted_pt)  # Convert distorted_pt to a 2d array so it can be treated as a set of points

	return cv2.undistortPoints(np.array([distorted_pt], dtype=float), camera_matrix, dist_coefs, P=camera_matrix).squeeze()

def undist_to_dist_img_coords(undistorted_pt, camera_matrix_or_calib_file, dist_coefs=None):
	"""
	Given the *undistorted* pixel coordinates (u_undist, v_undist), retrieve the pixel location on
	*distorted* image coordinates (u_dist, v_dist). It is the reverse operation of cv2.undistortPoints.
	:param undistorted_pt: Nx2 np.array with the location of point(s) in *undistorted* image coordinates (u_undist, v_undist).
	:param camera_matrix_or_calib_file: If str, indicates the name of the calibration file to load camera_matrix and dist_coefs from.
	Otherwise, should be a 3x3 np.matrix containing the intrisics of the camera model.
	:param dist_coefs: 1x5 np.array containing the distortion coefficients of the lens [k1, k2, p1, p2, k3]
	(ignored if camera_matrix_or_calib_file is str).
	:return: Nx2 np.array with the location of undistorted_pt in *distorted* image coordinates (u_dist, v_dist).
	"""
	camera_matrix, dist_coefs = get_cam_matrix_and_dist_coefs(camera_matrix_or_calib_file, dist_coefs)
	undistorted_pt = np.matrix(undistorted_pt)  # Convert undistorted_pt to a 2d array so it can be treated as a set of points
	distorted_pt = np.empty_like(undistorted_pt)  # Allocate memory for distorted points

	for i in range(0, undistorted_pt.shape[0]):  # For every point:
		# Simply reverse the operation performed by InitUndistortRectifyMap, which is documented here:
		# http://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html#cv.InitUndistortRectifyMap
		norm_undist_pt = np.array((undistorted_pt[i,:] - camera_matrix[0:2, 2]) / np.diag(camera_matrix)[0:2]).ravel()
		r2 = np.dot(norm_undist_pt, norm_undist_pt.T)  # Norm squared
		norm_dist_pt = norm_undist_pt * (1 + dist_coefs[0]*r2 + dist_coefs[1]*r2*r2 + dist_coefs[4]*r2*r2*r2)
		norm_dist_pt[0] += 2*dist_coefs[2]*np.prod(norm_undist_pt) + dist_coefs[3]*(r2 + 2*norm_undist_pt[0]*norm_undist_pt[0])
		norm_dist_pt[1] += dist_coefs[2]*(r2 + 2*norm_undist_pt[1]*norm_undist_pt[1]) + 2*dist_coefs[3]*np.prod(norm_undist_pt)
		distorted_pt[i,:] = np.array(camera_matrix * np.matrix(np.hstack((norm_dist_pt, 1))).T).ravel()[0:-1]

	return distorted_pt.squeeze()  # Squeeze the set of points so in case input was 1d, output is also 1d

def img_to_cam_coords(img_coords, camera_matrix_or_calib_file, dist_coefs=None, is_dist_img_coords=True):
	"""
	Converts image coordinates (in px) to camera coordinates (in m) [undistorts img coords first if necessary].
	Image coords start at the top-left corner of the image, and have the x axis increase right-wards; y axis down-wards.
	Camera coords start at camera's location, and have z axis pointing in their viewing direction (increases forward);
	x axis increases right-wards; y axis down-wards (parallel to image coordinates x-y axes).
	:param img_coords: Nx3 np.array: for each row, first 2 elements are the location of a point in image coordinates (in px);
	3rd element is the distance (in m) that the point is away from the camera.
	:param camera_matrix_or_calib_file: If str, indicates the name of the calibration file to load camera_matrix and dist_coefs from.
	Otherwise, should be a 3x3 np.matrix containing the intrisics of the camera model.
	:param dist_coefs: 1x5 np.array containing the distortion coefficients of the lens [k1, k2, p1, p2, k3]
	(ignored if camera_matrix_or_calib_file is str).
	:param is_dist_img_coords: True if provided img_coords are *distorted*; False if img_coords are *undistorted*.
	:return: Nx3 np.array with the location of the given point(s) in camera coordinates (in m).
	"""
	camera_matrix, dist_coefs = get_cam_matrix_and_dist_coefs(camera_matrix_or_calib_file, dist_coefs)
	img_coords = np.matrix(img_coords)  # Convert img_coords to a 2d array so it can be treated as a set of points
	if is_dist_img_coords:  # If img_coords are distorted, need to undistort first (leave depth untouched)!
		img_coords[:, 0:2] = dist_to_undist_img_coords(img_coords[:, 0:2], camera_matrix, dist_coefs)

	# For matrix multiplication, we'll need to transpose the pixel coordinates in img_coords (so it's a 2xN matrix).
	# Then, we add a row of (N) 1s to make coords homogeneous (so matrix multiplication can add translation)
	img_coords_homog = np.vstack((img_coords[:, 0:2].T, np.ones(img_coords.shape[0])))

	# Then, use the inverse camera matrix to transform img->cam coords (normalized), and transpose back to Nx2, which was the original format
	norm_cam_coords = (np.linalg.inv(camera_matrix) * img_coords_homog).T

	# Finally, multiply each row by its depth (img_coords[:,2]) to get the actual cam_coords (in m).
	return np.array(np.multiply(img_coords[:, 2], norm_cam_coords)).squeeze()  # Squeeze the set of points so in case input was 1d, output is also 1d

def cam_to_img_coords(cam_coords, camera_matrix_or_calib_file, dist_coefs=None, want_dist_img_coords=True):
	"""
	Converts camera coordinates (in m) to image coordinates (in px) [distorts img coords if requested].
	Image coords start at the top-left corner of the image, and have the x axis increase right-wards; y axis down-wards.
	Camera coords start at camera's location, and have z axis pointing in their viewing direction (increases forward);
	x axis increases right-wards; y axis down-wards (parallel to image coordinates x-y axes).
	:param cam_coords: Nx3 np.array: each row represents a given point in camera coordinates (in m).
	:param camera_matrix_or_calib_file: If str, indicates the name of the calibration file to load camera_matrix and dist_coefs from.
	Otherwise, should be a 3x3 np.matrix containing the intrisics of the camera model.
	:param dist_coefs: 1x5 np.array containing the distortion coefficients of the lens [k1, k2, p1, p2, k3]
	(ignored if camera_matrix_or_calib_file is str).
	:param want_dist_img_coords: True if want the output img_coords to be *distorted*; False for *undistorted*.
	:return: Nx2 np.array with the location of the given point(s) in image coordinates (in px).
	"""
	camera_matrix, dist_coefs = get_cam_matrix_and_dist_coefs(camera_matrix_or_calib_file, dist_coefs)
	cam_coords = np.matrix(cam_coords)  # Convert cam_coords to a 2d array so it can be treated as a set of points

	# For matrix multiplication, we'll need to transpose the coordinates in cam_coords (so it's a 3xN matrix).
	img_coords_homog = camera_matrix * cam_coords.T

	# Then, apply the homogeneous transform (divide by each point's 3rd coordinate) to get the *undistorted* img_coords in px (and transpose back to Nx2, which was the original format)
	img_coords = np.divide(img_coords_homog[0:2, :], img_coords_homog[2, :]).T

	# Finally, distort if requested
	if want_dist_img_coords:  # If we want img_coords to be distorted, apply distortion to (undistorted) img_coords
		img_coords = undist_to_dist_img_coords(img_coords, camera_matrix, dist_coefs)
	return np.array(img_coords).squeeze()  # Squeeze the set of points so in case input was 1d, output is also 1d

def cam_to_world_coords(cam_coords, world_to_camera_transf):
	"""
	Converts camera coordinates (in m) to world coordinates (in m).
	Camera coords start at camera's location, and have z axis pointing in their viewing direction (increases forward);
	x axis increases right-wards; y axis down-wards (parallel to image coordinates x-y axes).
	World coords start at 1st point of the calibration grid, and follow the axes described by the grid (see get_calib_pattern_info's pattern_points).
	:param cam_coords: Nx3 np.array: each row represents a given point in camera coordinates (in m).
	:param world_to_camera_transf: 3x4 extrinsic matrix from the camera model (indicates world->cam transform).
	:return: Nx3 np.array with the location of the given point(s) in world coordinates (in m).
	"""
	cam_coords = np.matrix(cam_coords)  # Convert cam_coords to a 2d array so it can be treated as a set of points
	camera_to_world_transf = get_cam_to_world_transform(world_to_camera_transf)  # Invert world->cam transf to get cam->world

	# For matrix multiplication, we'll need to transpose the coordinates in cam_coords (so it's a 3xN matrix).
	# Then, we add a row of (N) 1s to make coords homogeneous (so matrix multiplication can add translation)
	cam_coords_homog = np.vstack((cam_coords.T, np.ones(cam_coords.shape[0])))

	# Finally, apply cam->world transform and transpose the result back to Nx3, which was the original format
	world_coords = (camera_to_world_transf * cam_coords_homog).T

	return np.array(world_coords).squeeze()  # Squeeze the set of points so in case input was 1d, output is also 1d

def world_to_cam_coords(world_coords, world_to_camera_transf):
	"""
	Converts world coordinates (in m) to camera coordinates (in m).
	Camera coords start at camera's location, and have z axis pointing in their viewing direction (increases forward);
	x axis increases right-wards; y axis down-wards (parallel to image coordinates x-y axes).
	World coords start at 1st point of the calibration grid, and follow the axes described by the grid (see get_calib_pattern_info's pattern_points).
	:param world_coords: Nx3 np.array: each row represents a given point in camera coordinates (in m).
	:param world_to_camera_transf: 3x4 extrinsic matrix from the camera model (indicates world->cam transform).
	:return: Nx3 np.array with the location of the given point(s) in world coordinates (in m).
	"""
	world_coords = np.matrix(world_coords)  # Convert world_coords to a 2d array so it can be treated as a set of points

	# For matrix multiplication, we'll need to transpose the coordinates in world_coords (so it's a 3xN matrix).
	# Then, we add a row of (N) 1s to make coords homogeneous (so matrix multiplication can add translation)
	world_coords_homog = np.vstack((world_coords.T, np.ones(world_coords.shape[0])))

	# Finally, apply world->cam transform and transpose the result back to Nx3, which was the original format
	cam_coords = (world_to_camera_transf * world_coords_homog).T

	return np.array(cam_coords).squeeze()  # Squeeze the set of points so in case input was 1d, output is also 1d

def get_calib_pattern_info(is_chessboard, pattern_grid_or_cell_size=0.02, pattern_points=None):
	"""
	Retrieve information (pattern grid size and point location) about a calibration pattern.
	:param is_chessboard: True if the calibration pattern is a chessboard. False, for asymmetric circle grid.
	:param pattern_grid_or_cell_size: If not a tuple, indicates the distance (in m) between two consecutive points in the pattern grid;
	If a tuple, it already indicates the pattern grid and pattern_grid_or_cell_size and pattern_points will be returned unmodified.
	:param pattern_points: If pattern_grid_or_cell_size is not a tuple, it has no effect (leave as None);
	Otherwise, should be a Nx3 np.array containing the location of each point of the grid in world coordinates.
	:return: pattern_grid: Tuple indicating the number of dots in each dimension of the pattern grid, and
	pattern_points: Nx3 np.array containing the location of each point of the grid in world coordinates.
	"""
	if not isinstance(pattern_grid_or_cell_size, tuple):  # If pattern_grid_or_cell_size is not a tuple, it indicates the calibration pattern cell size (in m)
		cell_size = pattern_grid_or_cell_size
		pattern_grid = (9, 6) if is_chessboard else (4, 11)  # OpenCV's predefined calibration patterns
		pattern_points = np.zeros((np.prod(pattern_grid), 3), np.float32)
		pattern_points[:, :2] = np.indices(pattern_grid).T.reshape(-1, 2)  # [[0,0], [1,0], [2,0]...]
		if not is_chessboard:  # Asymmetric circular pattern follows [2*i + j%2, j]
			pattern_points[:,0] = pattern_points[:,0]*2 + (pattern_points[:,1]%2)
		pattern_points *= cell_size  # Convert units to meters, since we know the size of a cell
	else:  # If pattern_grid_or_cell_size is a tuple, then it's the pattern grid itself. Do nothing.
		pattern_grid = pattern_grid_or_cell_size

	return pattern_grid, pattern_points

def find_calib_pattern(img, is_chessboard, pattern_grid_or_cell_size=0.02):
	"""
	Look for a calibration pattern in the image provided, and return its location if found.
	:param img: Input image where the calibration pattern wants to be found.
	:param is_chessboard: True if the calibration pattern is a chessboard. False, for asymmetric circle grid.
	:param pattern_grid_or_cell_size: If a tuple, indicates the number of dots in each dimension of the pattern grid;
	Otherwise, it indicates the distance (in m) between two consecutive points in the pattern grid.
	:return: found: True/False indicating whether the calibration pattern was found in the image provided; and
	corners: 3d (nx1x2) np.array containing the location of each point of the grid found in image coordinates.
	"""
	pattern_grid, _ = get_calib_pattern_info(is_chessboard, pattern_grid_or_cell_size)  # In case they don't want to provide pattern_grid, automatically compute it based on is_chessboard

	# Use cv2 predefined functions to find either chessboard corners or circles
	if is_chessboard:
		found, corners = cv2.findChessboardCorners(img, pattern_grid)
		if found:  # Refine corner location
			cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1))
	else:
		found, corners = cv2.findCirclesGrid(img, pattern_grid, flags=cv2.CALIB_CB_ASYMMETRIC_GRID + cv2.CALIB_CB_CLUSTERING)

	return found, corners

def find_world_to_cam_and_F(img, camera_matrix_or_calib_file, dist_coefs=None, is_chessboard=False, pattern_grid_or_cell_size=0.02, pattern_points=None):
	"""
	Looks for the calibration pattern in the given image and computes the world->cam transform as well as the camera's focal length.
	:param img: Input image where the calibration pattern wants to be found.
	:param camera_matrix_or_calib_file: If str, indicates the name of the calibration file to load camera_matrix and dist_coefs from.
	Otherwise, should be a 3x3 np.matrix containing the intrisics of the camera model.
	:param dist_coefs: 1x5 np.array containing the distortion coefficients of the lens [k1, k2, p1, p2, k3]
	(ignored if camera_matrix_or_calib_file is str).
	:param is_chessboard: True if the calibration pattern is a chessboard. False, for asymmetric circle grid.
	:param pattern_grid_or_cell_size: If not a tuple, indicates the distance (in m) between two consecutive points in the pattern grid;
	If a tuple, it already indicates the pattern grid and pattern_grid_or_cell_size and pattern_points will be returned unmodified.
	:param pattern_points: If pattern_grid_or_cell_size is not a tuple, it has no effect (leave as None);
	Otherwise, should be a Nx3 np.array containing the location of each point of the grid in world coordinates.
	:return: world_to_camera_transf: 3x4 extrinsic matrix from the camera model (indicates world->cam transform); and
	F: Focal length of the camera: apparent length (in px) of an object that measures "x" m and is placed "x" m away from the camera.
	"""
	world_to_camera_transf = F = None  # Initialize default values
	camera_matrix, dist_coefs = get_cam_matrix_and_dist_coefs(camera_matrix_or_calib_file, dist_coefs)
	pattern_grid, pattern_points = get_calib_pattern_info(is_chessboard, pattern_grid_or_cell_size, pattern_points)

	found, corners = find_calib_pattern(img, is_chessboard, pattern_grid)  # Look for calibration pattern
	if found:
		world_to_camera_transf = find_world_to_cam_transform(pattern_points, corners, camera_matrix, dist_coefs)

		# In order to compute F, we will measure the distance between the first 2 pattern_points, both in px and m
		undist_corners = dist_to_undist_img_coords(corners[0:2], camera_matrix, dist_coefs)  # First, undistort the location of the first 2 pattern_points in px (corners[0] and corners[1])
		radius_in_px = np.linalg.norm(undist_corners[1,:] - undist_corners[0,:])
		radius_in_m = np.linalg.norm(pattern_points[1]-pattern_points[0])
		dist_in_m = world_to_cam_coords((pattern_points[1]-pattern_points[0])/2, world_to_camera_transf)[2]  #Measure the distance to the middle point between the first 2 pattern points
		F = radius_in_px*dist_in_m/radius_in_m

	return world_to_camera_transf, F

def find_world_to_cam_transform(world_points, img_points, camera_matrix_or_calib_file, dist_coefs=None, is_dist_img_coords=True):
	"""
	Computes the optimal world->cam transform from a set of corresponding points in world and image coordinates.
	:param world_points: Nx3 np.array containing the location of a set of known points in world coordinates.
	:param img_points: Nx3 np.array containing the location of a set of known points in image coordinates.
	:param camera_matrix_or_calib_file: If str, indicates the name of the calibration file to load camera_matrix and dist_coefs from.
	Otherwise, should be a 3x3 np.matrix containing the intrisics of the camera model.
	:param dist_coefs: 1x5 np.array containing the distortion coefficients of the lens [k1, k2, p1, p2, k3]
	(ignored if camera_matrix_or_calib_file is str).
	:param is_dist_img_coords: True if provided img_points are *distorted*; False if img_points are *undistorted*.
	:return: 3x4 extrinsic matrix from the camera model (indicates world->cam transform).
	"""
	camera_matrix, dist_coefs = get_cam_matrix_and_dist_coefs(camera_matrix_or_calib_file, dist_coefs)
	if not is_dist_img_coords:  # img_coords need to be in *distorted* image coords. If undistorted, distort first.
		img_points = undist_to_dist_img_coords(img_points, camera_matrix, dist_coefs)

	_, rvecs, tvec = cv2.solvePnP(world_points, img_points, camera_matrix, dist_coefs)
	rotation_matrix = np.matrix(cv2.Rodrigues(rvecs)[0])
	translation_vector = np.matrix(tvec)

	return np.matrix(np.hstack((rotation_matrix, translation_vector)))  # Compose the world->cam transformation matrix

def get_cam_to_world_transform(world_to_camera_transf):
	"""
	Obtains the cam->world transformation matrix from the world->cam matrix.
	:param world_to_camera_transf: 3x4 extrinsic matrix from the camera model (indicates world->cam transform).
	:return: 3x4 matrix with the cam->world transformation matrix (the inverse transformation of world_to_camera_transf)
	"""
	# This is just the inverse of world->cam_transf. So we invert the rotation first (transpose of the world->cam rotation matrix)
	rotation_matrix_inv = world_to_camera_transf[0:3, 0:3].T

	# And then invert the translation (which is just a subtraction: -original_translation), after undoing the rotation
	translation_vector_inv = -rotation_matrix_inv * np.matrix(world_to_camera_transf)[:,3]

	return np.matrix(np.hstack((rotation_matrix_inv, translation_vector_inv)))  # Compose the cam->world transformation matrix

def get_rvecs_and_tvec(world_to_camera_transf):
	"""
	Obtains rvecs and tvec from the world->cam transform matrix (reverse step of find_world_to_cam_transform)
	:param world_to_camera_transf: 3x4 extrinsic matrix from the camera model (indicates world->cam transform).
	:return: 3x1 np.array rvecs (rotation vector) and 3x1 (translation vector) to transform world to camera points.
	"""
	# Use Rodrigues to convert back and forth between rotation matrix and rvecs. Obtain the rotation matrix from the extrinsic matrix (world->camera transform)
	rvecs, _ = cv2.Rodrigues(np.matrix(world_to_camera_transf[0:3, 0:3], float))

	# tvec is simply the translation component (4th column) of the extrinsic matrix (world->camera transform)
	tvec = np.matrix(world_to_camera_transf)[:,3]  # Convert to np.matrix so it stays as a column vector instead of row

	return rvecs, tvec



