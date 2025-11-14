# conteoFresas.py
from ultralytics import YOLO


def contar_fresas(img_color, detector, areaMin=300, areaMax=60000, confMin=0.55):
    """
    Regresa:
       num_fresas, lista_bboxes
    """
    resultados = detector(img_color, verbose=False)[0]
    fresas = []

    for det in resultados.boxes:
        cls = int(det.cls[0])
        conf = det.conf[0]
        etiqueta = detector.names[cls]

        if etiqueta not in ["FRESA_ROJA"]:
            continue
        if conf < confMin:
            continue

        x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())
        area = (x2 - x1) * (y2 - y1)

        if areaMin < area < areaMax:
            fresas.append((x1, y1, x2, y2))

    return len(fresas), fresas


# Si quieres probar este archivo solo:
if __name__ == "__main__":
    import cv2
    detector = YOLO("env\\Lib\\best.pt")
    img = cv2.imread("test.jpg")
    n, _ = contar_fresas(img, detector)
    print("Fresas detectadas:", n)
