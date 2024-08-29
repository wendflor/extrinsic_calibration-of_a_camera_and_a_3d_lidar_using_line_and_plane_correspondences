import numpy as np
import open3d as o3d

# load point cloud from numpy array
point_cloud_path = "input_data/point_cloud_on_target.npy"
points = np.load(point_cloud_path).astype(np.float64)

# convert .npy point cloud to open3D format ply (.npy array datatype must bei float64!)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
print(np.asarray(pcd.points))
hull, _ = pcd.compute_convex_hull()
#plane = pcd.segment_plane(5,100000,1000)
#print(plane)
#pcd_voxel = pcd.voxel_down_sample(1)
#print(np.asarray(pcd_voxel.points))
o3d.visualization.draw_geometries([pcd], window_name="Original Point Cloud")
#o3d.visualization.draw_geometries([pcd_voxel], window_name="Voxel Point Cloud")
o3d.visualization.draw_geometries([hull], window_name="Convex Hull Only")

voxel_size = 1  

# 3. Runde die Punkte auf das nächstgelegene Raster
points = np.asarray(pcd.points)
rasterized_points = np.floor(points[:, :2] / voxel_size + 0.5) * voxel_size
rasterized_points = np.hstack((rasterized_points, points[:, 2:3]))  # Füge die z-Koordinate hinzu

# 4. Eliminiere Duplikate (halte nur einen Punkt pro Rasterzelle)
unique_points, indices = np.unique(rasterized_points[:, :2], axis=0, return_index=True)
rasterized_points = rasterized_points[indices]

boundaries, mask = pcd.compute_boundary_points(2, 5)
boundaries.paint_uniform_color([1.0, 0.0, 0.0])
o3d.visualization.draw([pcd, boundaries])


# 5. Erstelle eine neue Punktwolke mit den einzigartigen Punkten
rasterized_pcd = o3d.geometry.PointCloud()
rasterized_pcd.points = o3d.utility.Vector3dVector(rasterized_points)

print(np.asarray(rasterized_pcd.points))
np.save('point_cloud_raster_on_target_1mm.npy', np.asarray(rasterized_pcd.points).astype(np.float32) )
np.savetxt('point_cloud_raster_on_target_1mm.txt', np.asarray(rasterized_pcd.points).astype(np.float32) , fmt='%.6f')
# Optional: Visualisiere die rasterisierte Punktwolke
o3d.visualization.draw_geometries([rasterized_pcd], window_name="Rasterized Point Cloud")