import cv2
import os
import numpy as np

# Globale Variablen zum Speichern der Punkte und des Bildnamens
points = []
image_path = 'input_data/zed_calib/calib_zed.png'  
output_dir = 'input_data/zed_calib'

# Bild laden
image = cv2.imread(image_path)
if image is None:
    print("Bild konnte nicht geladen werden.")
    exit()

# Ursprünglichen Bildnamen ohne Erweiterung extrahieren
image_name = os.path.splitext(os.path.basename(image_path))[0]

# Maus-Callback-Funktion
def draw_rectangle(event, x, y, flags, param):
    global points, image
    
    if event == cv2.EVENT_LBUTTONDOWN:
        # Punkt zum Zeichnen speichern
        points.append((x, y))
        
        # Wenn vier Punkte gewählt wurden, zeichne das Viereck
        if len(points) == 4:
            # Viereck zeichnen (von Punkt zu Punkt) mit Antialiasing
            cv2.line(image, points[0], points[1], (0, 255, 255), 2, cv2.LINE_AA)
            cv2.line(image, points[1], points[2], (0, 255, 255), 2, cv2.LINE_AA)
            cv2.line(image, points[2], points[3], (0, 255, 255), 2, cv2.LINE_AA)
            cv2.line(image, points[3], points[0], (0, 255, 255), 2, cv2.LINE_AA)

            # Bild mit gezeichnetem Viereck anzeigen
            cv2.imshow('Image', image)

# Fenster erstellen und Maus-Callback setzen
cv2.namedWindow('Image')
cv2.setMouseCallback('Image', draw_rectangle)

print("Klicke 4 Punkte im Bild, um ein gelbes Viereck zu zeichnen. Drücke 's', um das Bild zu speichern.")

# Hauptschleife
while True:
    cv2.imshow('Image', image)
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC-Taste zum Beenden
        break
    elif key == ord('s'):  # 's' zum Speichern des Bildes
        # Speichern des Bildes im Verzeichnis input_data mit neuem Namen
        output_path = os.path.join(output_dir, f"{image_name}_yellow_edge.png")
        cv2.imwrite(output_path, image)
        print(f"Bild gespeichert als {output_path}")

cv2.destroyAllWindows()
