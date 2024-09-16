from copy import copy

import matplotlib.pyplot as plt
import numpy as np

from lidar_find_line_equation import map_point_to_line, ransac_line_in_lidar
from find_target_boundary_in_point_cloud import find_boundary_points_of_point_cloud
from lidar_find_plane import ransac_plane_in_lidar
from utils_display import show_point_cloud


def distance_of_points_to_plane(point_cloud, plane_equation):
    # distance of points in point cloud to the plane
    point_cloud_with_one = np.hstack((point_cloud, np.ones(shape=(point_cloud.shape[0], 1))))
    distance_points_to_plane = np.dot(point_cloud_with_one, plane_equation.T)

    return distance_points_to_plane


def map_points_to_plane(point_cloud, plane_equation):
    
    # find distance of points to the plane
    distance_points_to_plane = distance_of_points_to_plane(point_cloud=point_cloud, plane_equation=plane_equation)
    distance_points_to_plane = np.reshape(distance_points_to_plane, newshape=(-1, 1))

    projected_point_cloud = -1 * np.dot(distance_points_to_plane, np.reshape(plane_equation[0:3], newshape=(1,3))) + point_cloud

    return projected_point_cloud

'''
def find_different_lines_2(point_cloud, min_distance=None, maximim_distance_two_consecutive_points_in_ray=2):
    """
    point cloud
    min_distance: minimum distance for two points to be on the same line (mm). if it be None, it find it itself.
    """
    if min_distance is None:

        all_dis = []
        for row_i in range(0, point_cloud.shape[0]):
            dis = []
            for row_j in range(0, point_cloud.shape[0]):
                if row_i == row_j:
                    continue
                dis.append(np.linalg.norm(point_cloud[row_i]-point_cloud[row_j]))

            if len(dis) != 0:
                min_1 = np.min(dis)
                dis.remove(min_1)
                min_2 = np.min(dis)
                
                if min_1 > maximim_distance_two_consecutive_points_in_ray or min_2 > maximim_distance_two_consecutive_points_in_ray:
                    continue

                if min_1 *  9 < min_2:
                    all_dis.append(min_1)
                else:
                    all_dis.append(min_2)
                
        #print(all_dis)

        min_distance = np.max(all_dis)
    
        #print(min_distance)

    lines = []
    seen_point = [False] * point_cloud.shape[0]
    
    for row_i in range(point_cloud.shape[0]):

        if seen_point[row_i] == True:
            continue
        
        for line_idx, line in enumerate(lines):
            for row_j in range(len(line)):
            
                if np.linalg.norm(point_cloud[row_i]-line[row_j]) <= min_distance:
                    lines[line_idx].append(point_cloud[row_i])
                    seen_point[row_i] = True
                    break
        
        if seen_point[row_i] == False:
            lines.append([point_cloud[row_i]])

        lines_copy = []
        for line in lines:
            if len(line) > 5:
                lines_copy.append(np.array(line))

    return lines_copy


def find_different_lines(point_cloud, min_distance_between_lines=1):
    """
    point_cloud: calibration target point cloud
    min_distance_between_lines: minimum distance between two Lidar line in mm.
    """
    all_points = point_cloud.tolist()

    # sort according to z
    all_points = sorted(all_points, key = lambda x: x[2])

    lines = []
    line = []
    
    for point in all_points:
        if len(line) > 0:
            if (point[2] - line[-1][2]) > min_distance_between_lines:
                if len(line) > 4:
                    lines.append(np.array(line))
                line = []
        line.append(point) 
    
    if len(line) > 4:
        lines.append(np.array(line))
    line = []

    return lines

def find_points_on_left_right_border(lines):
    
    points_on_left_border = []
    points_on_right_border = []
    
    for line in lines:
        # sory by y
        line = sorted(line, key = lambda x: x[1])

        points_on_left_border.append(line[-1])
        points_on_right_border.append(line[0])

    all_points = points_on_left_border + points_on_right_border
    all_points = np.array(all_points)

    return {'left_points': points_on_left_border, 'right_points': points_on_right_border, 'border_point_cloud': all_points}

def find_upper_and_lower_points_on_edges(points_on_left_border, points_on_right_border):

    # sort points based in their z
    points_on_left_border = sorted(points_on_left_border, key = lambda x: x[2])
    points_on_right_border = sorted(points_on_right_border, key = lambda x: x[2])

    if len(points_on_left_border) < 4 or len(points_on_right_border) < 4:
        raise ValueError('Not enough points on borders to find lines.')

    for set_idx, points_set in enumerate([points_on_left_border, points_on_right_border]):

        upper_points = []
        lower_points = []
        
        angel_vectors = [-2]
        for i in range(1, len(points_set)-1):
            vec_1 = points_set[i+1] - points_set[i]
            vec_2 = points_set[i] - points_set[i - 1]

            cos_two_vector = np.dot(vec_1, vec_2.T)/(np.linalg.norm(vec_1) * np.linalg.norm(vec_2))

            angel_vectors.append(1-cos_two_vector)

        
        angel_vectors_copy = copy(angel_vectors)
        max_1 = np.argmax(angel_vectors_copy)
        angel_vectors_copy.remove(angel_vectors_copy[max_1])
        max_2 = np.argmax(angel_vectors_copy)
        
        if np.abs(max_1 - max_2) == 1:
            if max_1 > max_2:
                break_point = max_2
            else:
                break_point = max_1
        else:
            break_point = max_1

        for i in range(0, len(points_set)):

            if i <= break_point:
                lower_points.append(points_set[i])
            else:
                upper_points.append(points_set[i])
                
        if set_idx == 0:
            left_lower_points = np.copy(lower_points)
            left_upper_points = np.copy(upper_points)
        else:
            right_lower_points = np.copy(lower_points)
            right_upper_points = np.copy(upper_points)

    return {'left_lower_points': left_lower_points, 'left_upper_points': left_upper_points, 'right_lower_points': right_lower_points, 'right_upper_points': right_upper_points}
'''


def generate_point_line(line_equation):
    point_cloud = []
    for step in np.linspace(start=-1000, stop=1000, num=200):
        point = line_equation[0] + step * line_equation[1]
        point_cloud.append(point)
    point_cloud = np.array(point_cloud)

    return point_cloud

def find_edges_of_calibration_target_in_lidar(lidar_points, plane_equation, display=False):

    # convert to numpy
    point_cloud = np.copy(lidar_points)
    noisy_plane_points = np.copy(lidar_points)

    # map points to plane
    projected_point_cloud = map_points_to_plane(point_cloud=point_cloud, plane_equation=plane_equation)
    '''
    # find different lines
    #lines = find_different_lines(point_cloud=projected_point_cloud, maximim_distance_two_consecutive_points_in_ray=maximim_distance_two_consecutive_points_in_ray)
    lines = find_different_lines(point_cloud=projected_point_cloud, min_distance_between_lines=maximim_distance_two_consecutive_points_in_ray)
    noisy_lines = find_different_lines(point_cloud=noisy_plane_points, min_distance_between_lines=maximim_distance_two_consecutive_points_in_ray)

    # find equation of each line in 3D space
    lines_equations = []
    for line in lines:
        best_ratio_line = ransac_line_in_lidar(lidar_point=line)
        lines_equations.append(best_ratio_line['line_equation'])

    # map noisy points of each line to the found line
    point_cloud_mapped_on_lines = None
    list_point_mapped_on_lines = []
    
    for line_idx in range(len(lines)):
        new_line = map_point_to_line(lines[line_idx], lines_equations[line_idx])
        
        if point_cloud_mapped_on_lines is None:
            point_cloud_mapped_on_lines = np.copy(new_line)
        else:
            point_cloud_mapped_on_lines = np.vstack((point_cloud_mapped_on_lines, new_line))
        
        list_point_mapped_on_lines.append(new_line)
    '''
    # centroid of denoised points on calibration target in LiDAR space
    denoised_plane_centroid = np.mean(projected_point_cloud, axis=0)
    denoised_plane_points = np.copy(projected_point_cloud)

    # find point on upper and lower edges
    denoised_edges_points = find_boundary_points_of_point_cloud(projected_point_cloud)
    noisy_edges_points = find_boundary_points_of_point_cloud(noisy_plane_points)

    # find equation of edges
    best_ratio_line_left_lower = ransac_line_in_lidar(lidar_point=denoised_edges_points['left_lower_points'])
    best_ratio_line_left_upper = ransac_line_in_lidar(lidar_point=denoised_edges_points['left_upper_points'])
    best_ratio_line_right_lower = ransac_line_in_lidar(lidar_point=denoised_edges_points['right_lower_points'])
    best_ratio_line_right_upper = ransac_line_in_lidar(lidar_point=denoised_edges_points['right_upper_points'])

    denoised_edges_centroid = {'left_lower_points': np.mean(denoised_edges_points['left_lower_points'], axis=0),
                      'left_upper_points': np.mean(denoised_edges_points['left_upper_points'], axis=0),
                      'right_lower_points': np.mean(denoised_edges_points['right_lower_points'], axis=0), 
                      'right_upper_points': np.mean(denoised_edges_points['right_upper_points'], axis=0)}

    left_lower_equation = best_ratio_line_left_lower['line_equation']
    left_upper_equation = best_ratio_line_left_upper['line_equation']
    right_lower_equation = best_ratio_line_right_lower['line_equation']
    right_upper_equation = best_ratio_line_right_upper['line_equation']

    # unify edge directions of calibration target inside lidar coordinate system
    if left_lower_equation[1][1] < 0:
        left_lower_equation[1] *= -1
    if left_upper_equation[1][1] > 0:
        left_upper_equation[1] *= -1
    if right_upper_equation[1][1] > 0:
        right_upper_equation[1] *= -1
    if right_lower_equation[1][1] < 0:
        right_lower_equation[1] *= -1

    plt_images = {}
    plt_img = show_point_cloud(point_cloud=point_cloud, title='Input Point Cloud')
    plt_images['input_point_cloud'] = np.copy(plt_img)
    plt_img = show_point_cloud(point_cloud=projected_point_cloud, title='Point Cloud Projected on Plane Equation')
    plt_images['point_cloud_projected_on_plane_equation'] = np.copy(plt_img)
    '''
    plt_img = show_point_cloud(point_cloud=[point_cloud_mapped_on_lines, dic_point_border['border_point_cloud']], marker='o', title='Points on Calibration Target Edges')
    plt_images['points_on_calibration_target_edges'] = np.copy(plt_img)
    '''
    plt_img = show_point_cloud(point_cloud=[denoised_edges_points['left_lower_points'], denoised_edges_points['left_upper_points'], denoised_edges_points['right_lower_points'], denoised_edges_points['right_upper_points']], marker='o', title='Left Lower, Left Upper, Right Lower and Right Upper Points on Edges')
    plt_images['points_on_edges'] = np.copy(plt_img)

    left_lower_pointcloud = generate_point_line(line_equation=left_lower_equation)
    left_upper_pointcloud = generate_point_line(line_equation=left_upper_equation)
    right_lower_pointcloud = generate_point_line(line_equation=right_lower_equation)
    right_upper_pointcloud = generate_point_line(line_equation=right_upper_equation)
    plt_img = show_point_cloud(point_cloud=[point_cloud, left_lower_pointcloud, left_upper_pointcloud, right_lower_pointcloud, right_upper_pointcloud], title='Equation of Edges')
    plt_images['edges_line_equtions_and_all_points'] = np.copy(plt_img)
    
    if display == True:
        for key_img in plt_images:
            plt.figure()
            plt.imshow(plt_images[key_img])
        plt.show()

    all_edges_equations = {'line_equation_left_lower': left_lower_equation, 'line_equation_left_upper': left_upper_equation,
            'line_equation_right_lower': right_lower_equation, 'line_equation_right_upper': right_upper_equation}

    return all_edges_equations, denoised_plane_centroid, denoised_edges_centroid, plt_images, denoised_plane_points, denoised_edges_points, noisy_plane_points, noisy_edges_points

if __name__ == '__main__':
    point_cloud = np.load('input_data/Visionerf_calib/point_cloud_on_target_0000.npy')
    

    best_ratio_plane = ransac_plane_in_lidar(point_cloud)
    plane_equation=best_ratio_plane['plane_equation']
    edges_equations = find_edges_of_calibration_target_in_lidar(point_cloud, plane_equation, display=True)[0]
    print(edges_equations)