import numpy as np
import open3d as o3d

def find_boundary_points_of_point_cloud(points, display=False):
    pcd = o3d.t.geometry.PointCloud(points)

    pcd.estimate_normals(max_nn=30, radius=20)
    

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
    # Set exclusion radius (20 mm)
    exclusion_radius = 20

    def is_within_radius(point, corner, radius):
        return np.linalg.norm(point - corner) <= radius

    # Create a mask to exclude points near any of the corner points using vectorized operations
    mask_exclude = np.array([
        not (is_within_radius(pt[:2], top_corner, exclusion_radius) or
             is_within_radius(pt[:2], left_corner, exclusion_radius) or
             is_within_radius(pt[:2], bottom_corner, exclusion_radius) or
             is_within_radius(pt[:2], right_corner, exclusion_radius))
        for pt in boundary_points
    ])
    # Apply the mask to exclude points within the exclusion radius
    filtered_points = boundary_points[mask_exclude]



    # Listen für die Punkte in jedem Bereich
    left_upper_points = []
    left_lower_points = []
    right_lower_points = []
    right_upper_points = []

    # Zuordnung der Punkte zu den Bereichen basierend auf ihren x- und y-Werten
    for point in filtered_points:
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
    left_upper_points = np.array(left_upper_points).astype(np.float64)
    left_lower_points = np.array(left_lower_points).astype(np.float64)
    right_lower_points = np.array(right_lower_points).astype(np.float64)
    right_upper_points = np.array(right_upper_points).astype(np.float64)
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
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        boundarys.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        o3d.visualization.draw_geometries([boundarys.to_legacy(), pcd.to_legacy()])

        # Create Open3D point clouds for each region
        pcd_left_upper = o3d.geometry.PointCloud()
        pcd_left_upper.points = o3d.utility.Vector3dVector(left_upper_points)
        # Assign a uniform color to all points (e.g., red [1.0, 0.0, 0.0])
        colors_lu = np.tile([1.0, 0.0, 0.0], (left_upper_points.shape[0], 1)).astype(np.float64)
        # Set the colors to the point cloud
        pcd_left_upper.colors = o3d.utility.Vector3dVector(colors_lu)    

        pcd_left_lower = o3d.geometry.PointCloud()
        pcd_left_lower.points = o3d.utility.Vector3dVector(left_lower_points)
        # Assign a uniform color to all points (green [0.0, 1.0, 0.0])
        colors_ll = np.tile([0.0, 1.0, 0.0], (left_lower_points.shape[0], 1)).astype(np.float64)
        # Set the colors to the point cloud
        pcd_left_lower.colors = o3d.utility.Vector3dVector(colors_ll) 

        pcd_right_lower = o3d.geometry.PointCloud()
        pcd_right_lower.points = o3d.utility.Vector3dVector(right_lower_points)
        # Assign a uniform color to all points ( yellow [1.0, 1.0, 0.0])
        colors_rl = np.tile([1.0, 1.0, 0.0], (right_lower_points.shape[0], 1)).astype(np.float64)
        # Set the colors to the point cloud
        pcd_right_lower.colors = o3d.utility.Vector3dVector(colors_rl)        

        pcd_right_upper = o3d.geometry.PointCloud()
        pcd_right_upper.points = o3d.utility.Vector3dVector(right_upper_points)
        # Assign a uniform color to all points ( blue [0.0, 0.0, 1.0])
        colors_ru = np.tile([0.0, 0.0, 1.0], (right_upper_points.shape[0], 1)).astype(np.float64)
        # Set the colors to the point cloud
        pcd_right_upper.colors = o3d.utility.Vector3dVector(colors_ru)       

        # Visualize all point clouds together
        o3d.visualization.draw_geometries([pcd_left_upper, pcd_left_lower, pcd_right_lower, pcd_right_upper])





    return {'left_lower_points': left_lower_points, 'left_upper_points': left_upper_points, 'right_lower_points': right_lower_points, 'right_upper_points': right_upper_points}

if __name__ == '__main__':  


    # load point cloud from numpy array
    #point_cloud_path = "extrinsic_calibration-of_a_camera_and_a_3d_lidar_using_line_and_plane_correspondences/input_data/Visionerf_calib/point_cloud_on_target_0005.npy" # Debugger path
    point_cloud_path = "input_data/Visionerf_calib/point_cloud_on_target_0000.npy"
    point_cloud = np.load(point_cloud_path).astype(np.float64)
    noisy_edge_points_dic = find_boundary_points_of_point_cloud(point_cloud, display =True)
    print(noisy_edge_points_dic)