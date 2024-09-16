import numpy as np
import os

def crop_point_cloud(input_file_path, max_depth):
    # Read the point cloud from the binary file
    point_cloud = np.fromfile(input_file_path, dtype=np.float32)
    points = point_cloud[1:].reshape(-1, 3)  # Expecting XYZ-Pointcloud

    # Filter the point cloud based on the Z value
    filtered_points = points[(points[:, 2] <= max_depth)]

    # Extract the number from the input file path
    file_name = os.path.basename(input_file_path)  # Get the file name
    number = ''.join(filter(str.isdigit, file_name))  # Extract the number from the file name

    # Create the output file paths based on the extracted number
    output_file_path_scene = f'input_data/Visionerf_calib/point_cloud_whole_scene_{number}.npy'
    output_file_path_cropped = f'input_data/Visionerf_calib/point_cloud_on_target_{number}.npy'

    # Save the point clouds as .npy files
    np.save(output_file_path_scene, points)
    np.save(output_file_path_cropped, filtered_points)
    print(f"Point Clouds saved as .npy-arrays in '{output_file_path_cropped}' and '{output_file_path_scene}'.")

    # Create the dictionary based on the extracted number
    point_cloud_dict = {
        f'point_cloud_whole_scene_{number}': points,
        f'point_cloud_on_target_{number}': filtered_points
    }

    return point_cloud_dict

if __name__ == '__main__':
    input_file_path = 'input_data/Visionerf_calib/visionerf_calib_0000.bin'  # PATH to .bin-file
    # Filter point cloud w.r.t. Z-Value (for 0000 --> 530, others ---> 455)
    max_depth = 530
    point_clouds = crop_point_cloud(input_file_path, max_depth)
    print(point_clouds)
    file_name = os.path.basename(input_file_path)  # Get the file name
    number = ''.join(filter(str.isdigit, file_name))  # Extract the number from the file name

    whole_scene = point_clouds[f'point_cloud_whole_scene_{number}']
    on_target = point_clouds[f'point_cloud_on_target_{number}']
    print("Point Cloud - Whole Scene:\n", whole_scene)
    print("\nPoint Cloud - On Target:\n", on_target)


