import numpy as np
import open3d as o3d

def find_boundary_points_of_point_cloud(points, display=False):
    pcd = o3d.t.geometry.PointCloud(points)
    pcd.estimate_normals(max_nn=50, radius=5)

    boundarys, mask = pcd.compute_boundary_points(5, 50)
    boundarys = boundarys.paint_uniform_color([1.0, 0.0, 0.0])
    '''
    print(pcd.to_legacy())
    '''
    print(f"Found Open3D boundary:'{boundarys.to_legacy()}'")
    pcd = pcd.paint_uniform_color([0.6, 0.6, 0.6])
    '''
    lookat = np.array( [0.0, 0.0, 0.0], np.float64)
    up = np.array( [-1.0, 0.0, 0.0], np.float64)
    front = np.array( [0.0, 0.0, 1.0], np.float64)
    zoom = 0.8
    ,lookat=lookat, up=up, front=front, zoom=zoom
    '''

    boundary_points = np.asarray(boundarys.point.positions.numpy())

    # Finden der Ecken
    x_min_index = np.argmin(boundary_points[:, 0])
    x_max_index = np.argmax(boundary_points[:, 0])
    y_min_index = np.argmin(boundary_points[:, 1])
    y_max_index = np.argmax(boundary_points[:, 1])

    # Festlegen der Ecken
    top_corner = boundary_points[x_min_index, :2]  # (x_min, zugehöriges y)
    left_corner = boundary_points[y_max_index, :2]  # (zugehöriges x, y_max)
    bottom_corner = boundary_points[x_max_index, :2]  # (x_max, zugehöriges y)
    right_corner = boundary_points[y_min_index, :2]  # (zugehöriges x, y_min)
    '''
    # Ausgabe der Ecken
    print(f"Top corner (x_min): {top_corner}")
    print(f"Left corner (y_max): {left_corner}")
    print(f"Bottom corner (x_max): {bottom_corner}")
    print(f"Right corner (y_min): {right_corner}")
    '''

    # Listen für die Punkte in jedem Bereich
    left_upper_points = []
    left_lower_points = []
    right_lower_points = []
    right_upper_points = []

    # Zuordnung der Punkte zu den Bereichen basierend auf ihren x- und y-Werten
    for point in boundary_points:
        x, y = point[0], point[1]
        
        # upper_left_points: Zwischen top_corner und left_corner
        if x >= top_corner[0] and x <= left_corner[0] and y <= left_corner[1] and y >= top_corner[1]:
            left_upper_points.append(point)
        
        # lower_left_points: Zwischen left_corner und bottom_corner
        elif x >= left_corner[0] and x <= bottom_corner[0] and y <= left_corner[1] and y >= bottom_corner[1]:
            left_lower_points.append(point)
        
        # lower_right_points: Zwischen bottom_corner und right_corner
        elif x >= right_corner[0] and x <= bottom_corner[0] and y <= bottom_corner[1] and y >= right_corner[1]:
            right_lower_points.append(point)
        
        # upper_right_points: Zwischen right_corner und top_corner
        elif x <= right_corner[0] and x >= top_corner[0] and y >= right_corner[1] and y <= top_corner[1]:
            right_upper_points.append(point)

    # Umwandeln in numpy arrays
    left_upper_points = np.array(left_upper_points)
    left_lower_points = np.array(left_lower_points)
    right_lower_points = np.array(right_lower_points)
    right_upper_points = np.array(right_upper_points)
    sum_all_points = len(left_upper_points)+ len(left_lower_points)+ len(right_lower_points)+ len(right_upper_points)
    '''
    # Ausgabe der Punkteanzahl in jeder Kategorie
    print(f"Upper Left Points: {len(left_upper_points)}")
    print(f"Lower Left Points: {len(left_lower_points)}")
    print(f"Lower Right Points: {len(right_lower_points)}")
    print(f"Upper Right Points: {len(right_upper_points)}")
    '''
    print(f"Total number of assigned points to edges: {sum_all_points}")
    
    if display == True:
        o3d.visualization.draw_geometries([boundarys.to_legacy(), pcd.to_legacy()])

    return {'left_lower_points': left_lower_points, 'left_upper_points': left_upper_points, 'right_lower_points': right_lower_points, 'right_upper_points': right_upper_points}

if __name__ == '__main__':  

    # load point cloud from numpy array
    point_cloud_path = "input_data/Visionerf_calib/point_cloud_on_target_170mm_18mm_08_30_16_20_22.npy"
    point_cloud = np.load(point_cloud_path)
    noisy_edge_points_dic = find_boundary_points_of_point_cloud(point_cloud, display =False)
    print(noisy_edge_points_dic)