import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
binary = True
file_path = 'input_data/Visionerf_calib/visionerf_calib_0000.bin'  # Pfad zu .bin-Datei
#file_path='input_data/Visionerf_calib/point_cloud_on_target_0002.npy'

if binary == True:
# Binärdatei einlesen
    
    point_cloud = np.fromfile(file_path, dtype=np.float32)


    points = point_cloud[1:].reshape(-1, 3)  # Annahme:  XYZ-Daten  
else :
    points= np.load(file_path)

# Filtere die Punkte basierend auf den Bedingungen für X und Y
filtered_points = points[(points[:, 2] <= 600)]

# X, Y, Z extrahieren
x = filtered_points[:, 0]
y = filtered_points[:, 1]
z = filtered_points[:, 2]

min_z = np.min(z)
max_z = np.max(z)

diff_z = max_z-min_z
print(diff_z)
# Matplotlib 3D-Plot erstellen
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Punktwolke plotten
ax.scatter(x, y, z, c=z, cmap='viridis', marker='o')  # Farbe basierend auf Z-Werten

# Blickrichtung auf die positive Z-Achse einstellen
ax.view_init(elev=-90, azim=180)

# Achsenlabel setzen
ax.set_xlabel('X Achse')
ax.set_ylabel('Y Achse')
ax.set_zlabel('Z Achse')

# Ursprung markieren
ax.scatter(0, 0, 0, color='red', s=100, label='Cirrus3D-300')
ax.legend()

# Plot anzeigen
plt.show()