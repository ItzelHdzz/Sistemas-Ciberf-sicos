import cv2 as cv
import numpy as np
import pyrealsense2 as rs
import time
from ultralytics import YOLO

# FUNCIÓN DE PROFUNDIDAD
def obtener_profundidad_media(img_profundidad, x1, y1, x2, y2, profEnmetros):
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(img_profundidad.shape[1] - 1, x2), min(img_profundidad.shape[0] - 1, y2)
    zona = img_profundidad[y1:y2+1, x1:x2+1]
    zona = zona[zona > 0]
    if zona.size == 0:
        return None
    lim_inf, lim_sup = np.percentile(zona, [5, 95])
    zona_filtrada = zona[(zona >= lim_inf) & (zona <= lim_sup)]
    if zona_filtrada.size == 0:
        return None
    return np.median(zona_filtrada) * profEnmetros

def main():
    # CONFIGURACIÓN DE LA CÁMARA REALSENSE
    camara = rs.pipeline()
    ajustes = rs.config()

    ajustes.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    ajustes.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    perfil_cam = camara.start(ajustes)
    alinear = rs.align(rs.stream.color)

    profStream = perfil_cam.get_stream(rs.stream.depth).as_video_stream_profile()
    colorStream = perfil_cam.get_stream(rs.stream.color).as_video_stream_profile()

    intrProf = profStream.get_intrinsics()
    intrColor = colorStream.get_intrinsics()
    profEnmetros = perfil_cam.get_device().first_depth_sensor().get_depth_scale()

    # MODELO YOLO
    detector = YOLO("env\\Lib\\best.pt")

    areaMin = 500
    areaMax = 50000
    fresaAncho = 0.0326
    fresaLarg = 0.0342

    # MATRIZ DE CÁMARA
    K = np.array([
        [intrColor.fx, 0, intrColor.ppx],
        [0, intrColor.fy, intrColor.ppy],
        [0, 0, 1]
    ], dtype=np.float64)

    dist = np.zeros(5, dtype=np.float32)

    # ESPERAR Y TOMAR FOTO
    print("Esperando que la cámara se estabilice...")
    for _ in range(30):  # descartar primeros frames
        camara.wait_for_frames()

    while True:
        print("\nTomando foto en 3 segundos...")
        time.sleep(3)  # Espera antes de tomar foto

        frames = camara.wait_for_frames()
        frames = alinear.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            print("No se detectan frames.")
            continue

        img_color = np.asanyarray(color_frame.get_data())
        img_depth = np.asanyarray(depth_frame.get_data())

        resultados = detector(img_color, verbose=False)[0]
        fresa_detectada = False

        for det in resultados.boxes:
            clase_id = int(det.cls[0])
            confianza = det.conf[0]
            etiqueta = detector.names[clase_id]
            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())
            area = (x2 - x1) * (y2 - y1)

            if confianza > 0.6 and areaMin < area < areaMax and etiqueta in ["FRESA_ROJA"]:
                fresa_detectada = True

                Z = obtener_profundidad_media(img_depth, x1, y1, x2, y2, profEnmetros)
                print("Distancia (m):", Z)

                if Z is None:
                    cv.rectangle(img_color, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cv.putText(img_color, f"{etiqueta} sin profundidad", (x1, y1 - 10),
                               cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                    continue

                puntos_img = np.array([
                    (x1, y1),
                    (x2, y1),
                    (x2, y2),
                    (x1, y2)
                ], dtype="double")

                puntos_obj = np.array([
                    (-fresaAncho/2, -fresaLarg/2, 0.0),
                    (fresaAncho/2, -fresaLarg/2, 0.0),
                    (fresaAncho/2, fresaLarg/2, 0.0),
                    (-fresaAncho/2, fresaLarg/2, 0.0)
                ])

                rvec = np.zeros((3, 1), dtype=np.float64)
                tvec = np.array([[0], [0], [Z]], dtype=np.float64)

                ok, rvec, tvec = cv.solvePnP(
                    objectPoints=puntos_obj,
                    imagePoints=puntos_img,
                    cameraMatrix=K,
                    distCoeffs=dist,
                    rvec=rvec,
                    tvec=tvec,
                    useExtrinsicGuess=True,
                    flags=cv.SOLVEPNP_ITERATIVE
                )

                color = (0, 255, 255) if ok else (0, 0, 255)
                cv.rectangle(img_color, (x1, y1), (x2, y2), color, 2)
                cv.putText(img_color, etiqueta, (x1, y1 - 10),
                           cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

                if ok:
                    tvec[2, 0] = Z
                    T = np.eye(4)
                    T[:3, :3], _ = cv.Rodrigues(rvec) 
                    T[:3, 3] = tvec.reshape(3)

                    print("Matriz de transformación:\n", T)
                    return T

        if not fresa_detectada:
            print("No se detectó ninguna fresa con suficiente confianza. Repitiendo...\n")
            continue

        cv.imshow("Foto usada para detección y PnP", img_color)
        cv.waitKey(0)
        break

    cv.destroyAllWindows()
    camara.stop()


if __name__ == "__main__":
    main()



