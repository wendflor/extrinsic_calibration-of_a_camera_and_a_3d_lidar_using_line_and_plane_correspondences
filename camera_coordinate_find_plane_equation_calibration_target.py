from tkinter.messagebox import NO

import cv2 as cv
import numpy as np

from lidar_find_plane import ransac_plane_in_lidar
from camera_image_find_calibration_target_in_camera import \
    find_corners_on_calibration_target
from read_calibration_file import read_yaml_file


def calculate_plane_equation_by_three_points(three_points):
    """
    The input is a 3 * 3 numpu array, each row is a point.
    It returns plane equation that passes those points: a numpy array with shape (4, )
    """
    # calculate plane equation ax+by+cz+d = 0
    vec_1 = three_points[1, :] - three_points[0, :]
    vec_2 = three_points[2, :] - three_points[0, :] 
    normal = np.cross(vec_1, vec_2)
    if normal[2] < 0: # vorher <0 
        normal *= -1
    normal /= np.linalg.norm(normal)
    d = -1 * (normal[0] * three_points[0, 0] + normal[1] * three_points[0, 1] + normal[2] * three_points[0, 2])
    plane_equation = np.array([normal[0], normal[1], normal[2], d])

    return plane_equation

def rotation_and_translation_of_target_in_camera_image(object_points, image_points, camera_matrix, distortion_coefficients):
    """
    object_points: Array of object points in the object coordinate space
    image_points: Array of corresponding image points
    camera_matrix: Input camera intrinsic matrix (3 in 3)
    distortion_coefficients: Input vector of distortion coefficients



    Output (rvec and tvec), rotation vector that, together with translation vector, brings points from the model coordinate system
    to the camera coordinate system. 
    rvec is in radian
    tvec is in mm
    """
    # retval, rvec, tvec = cv.solvePnP(objectPoints=object_points, imagePoints=image_points, cameraMatrix=camera_matrix, distCoeffs=distortion_coefficients)
    retval, rvec, tvec = cv.solvePnP(objectPoints=object_points, imagePoints=image_points, cameraMatrix=camera_matrix, distCoeffs=None)


    if retval == True:
        return {'rotation_vector': rvec, 'translation_vector': tvec}
    else:
        return None

def get_calibration_target_plane_equation_in_image(object_points, image_points, camera_matrix, distortion_coefficients):
    """
    Find plane equation of a calibration target inside and image. The plane equation is in camera coordinate system and in 
    homogenous format
    """

    rvec_tvec = rotation_and_translation_of_target_in_camera_image(object_points=object_points,
                                                       image_points=image_points,
                                                       camera_matrix=camera_matrix,
                                                       distortion_coefficients=distortion_coefficients)

    if rvec_tvec is None:
        return None
    
    # rotation vector (numpy array 3 * 1)
    rvec = rvec_tvec['rotation_vector']
    # translation vector (numpy array 3 * 1)
    tvec = rvec_tvec['translation_vector']

    # convert rotation vector to rotation matrix
    rotation_matrix, _ = cv.Rodrigues(src=rvec)

    # three points on calibration target obejct
    four_points = np.array([[0, 0, 0], [100, 0, 0], [100, 100, 0], [0, 100, 0]]) # before value*10

    # rotate and translate points from object to camera coordinate
    four_points = np.dot(rotation_matrix, four_points.T) + tvec
    four_points = four_points.T

    # calculate plane equation
    plane_equation = calculate_plane_equation_by_three_points(three_points=four_points[0:3, :])

    return plane_equation

def get_calibration_target_plane_equation_in_image_ransac(object_points, image_points, camera_matrix, distortion_coefficients):
    """
    Find plane equation of a calibration target inside and image. The plane equation is in camera coordinate system and in 
    homogenous format
    """

    rvec_tvec = rotation_and_translation_of_target_in_camera_image(object_points=object_points,
                                                       image_points=image_points,
                                                       camera_matrix=camera_matrix,
                                                       distortion_coefficients=distortion_coefficients)

    if rvec_tvec is None:
        return None
    
    # rotation vector (numpy array 3 * 1)
    rvec = rvec_tvec['rotation_vector']
    # translation vector (numpy array 3 * 1)
    tvec = rvec_tvec['translation_vector']

    # convert rotation vector to rotation matrix
    rotation_matrix, _ = cv.Rodrigues(src=rvec)

    # rotate and translate points from object to camera coordinate
    points_camera_coordinate = np.dot(rotation_matrix, object_points.T) + tvec
    points_camera_coordinate = points_camera_coordinate.T

    # calculate plane equation
    plane_equation = ransac_plane_in_lidar(points_camera_coordinate)

    return plane_equation


def camera_coordinate_plane_equation_calibration_target(rgb_img, num_row, num_col, square, camera_matrix, distortion_coefficients, display=False):
    """
    Find plane equation of a calibration target inside and image. The plane equation is in camera coordinate system and in 
    homogenous format
    """
    
    # find corners on calibration target
    points_3d_image_image_subpix = find_corners_on_calibration_target(
                                        img=rgb_img,
                                        num_row=num_row,
                                        num_col=num_col,
                                        square=square,
                                        display=display)

    if points_3d_image_image_subpix is None:
        raise ValueError('Can not find corners on checkerboard.')
    
    # find plane equation of calibtarion target inside image
    plane_equation = get_calibration_target_plane_equation_in_image(
                                object_points=points_3d_image_image_subpix['points_in_3D'],
                                image_points=points_3d_image_image_subpix['points_in_image_sub_pixel'],
                                camera_matrix=camera_matrix,
                                distortion_coefficients=distortion_coefficients
                            )
    
    return plane_equation

if __name__ == '__main__':
    

    ################################################################
    # Read image
    ################################################################
    # read image
    img_bgr = cv.imread('input_data/zed_calib/calib_zed_170mm_18mm_08_30_16_20.png')

    # convert BGR to RGB
    rgb_image = cv.cvtColor(img_bgr, cv.COLOR_BGR2RGB)

    ################################################################
    # calibration information related to camera
    ################################################################
    path = 'input_data/zed_calib/left_camera_calibration_parameters.yaml'
    calibration_data = read_yaml_file(path=path)
    print(calibration_data)

    plane_equation = camera_coordinate_plane_equation_calibration_target(
                            rgb_img=rgb_image,
                            num_row=6,
                            num_col=8,
                            square=18,
                            camera_matrix=calibration_data['camera_matrix'],
                            distortion_coefficients=calibration_data['distortion_coefficients'],
                            display=True
                        )

    print('Plane Equation:')
    print(plane_equation)
    
