import numpy as np

input_file_path = 'input_data/Visionerf_calib/XYZcloud_13_17_03.bin'  # Ersetze durch den Pfad zu deiner .bin-Datei
point_cloud = np.fromfile(input_file_path, dtype=np.float32)


points = point_cloud[1:].reshape(-1, 3)  # Annahme:  XYZ-Daten ab 

# Filtere die Punkte basierend auf den Bedingungen für X und Y
filtered_points = points[(points[:, 2] <= 550)]

output_file_path_scene = 'input_data/Visionerf_calib/point_cloud_whole_scene_13_17_03.npy'
output_file_path_cropped = 'input_data/Visionerf_calib/point_cloud_on_target_13_17_03.npy'

np.save(output_file_path_scene, points)
np.save(output_file_path_cropped, filtered_points)
print(f"Point Clouds saved as .npy-arrays in '{output_file_path_cropped}' and '{output_file_path_scene}'.") 