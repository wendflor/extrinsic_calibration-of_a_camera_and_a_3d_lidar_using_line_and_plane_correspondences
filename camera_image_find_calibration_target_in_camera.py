import glob

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

from utils_display import show_point_cloud 


def find_corners_on_calibration_target(img, num_row, num_col, square, display=False):
    """
    img: image that contain calibration target (RGB image)
    num_row: number of inside corners in row direction
    num_col: number of inside corners in col direction
    square: len of each calibration target square in mm

    returned points are sorted from left to right and top to bottom, also return results is in opencv format (x, y, z),
    x is in direction of horizon and y in direction of vertical axis
    """
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((num_row*num_col,3), np.float32)
    objp[:,:2] = np.mgrid[0:num_col,0:num_row].T.reshape(-1,2)
    objp *= square

    
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (num_col, num_row), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        # corners in subpixel space
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        
        img_copy = np.copy(img)
        cv.drawChessboardCorners(img_copy, (num_col, num_row), corners2, ret)
        
        corners = np.reshape(corners, newshape=(corners.shape[0], 2))
        corners2 = np.reshape(corners2, newshape=(corners2.shape[0], 2))

        if display:
            plt.figure(figsize=(10, 8))
            plt.imshow(img_copy)
            plt.title('Detected Corners')
            plt.show()

        return {'points_in_3D': objp, 'points_in_image': corners, 'points_in_image_sub_pixel': corners2, 'image_corners': img_copy}
    else:
        return None

if __name__ == '__main__':
    
    # read image from file
    img = cv.imread('input_data/zed_calib/calib_zed_08_23_13_17_yellow_edge.png')
    img = cv.cvtColor(img, cv.COLOR_BGR2RGB)

    # find corners on calibration target
    points_3d_image_image_subpix = find_corners_on_calibration_target(img=img, num_row=6, num_col=8, square=25, display=True)

    print(points_3d_image_image_subpix)
