"""
Código que realiza la captura de fotos automáticamente cada hora
"""
import cv2
import os
import time

# Inicializar cámara
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("No se pudo acceder a la cámara.")
    exit()

print("Cámara iniciada. Capturando una imagen cada hora. Presiona Ctrl+C para detener.")

# Directorio base para todas las carpetas
REP_BASE = "./repositorio"
LAST_DATE = "" # Variable para rastrear la última fecha usada

try:
    while True:
        # --- Lógica de Fecha y Hora (MOVIDA DENTRO DEL BUCLE) ---
        
        # 1. Obtener la hora actual
        current_time = time.localtime()
        
        # 2. Formatear la FECHA (para la carpeta) y la HORA (para el archivo)
        fecha = time.strftime("%Y-%m-%d", current_time)
        hora = time.strftime("%H-%M-%S", current_time)
        
        # 3. Crear la carpeta si la fecha ha cambiado
        if fecha != LAST_DATE:
            carpeta = os.path.join(REP_BASE, fecha)
            if not os.path.exists(carpeta):
                os.makedirs(carpeta)
            LAST_DATE = fecha # Actualiza la última fecha registrada

        # -----------------------------------------------------------

        # Lógica de Captura
        ret, frame = cap.read()
        if not ret:
            print("Error al capturar el frame.")
            break

        # 4. Construir nombres y rutas de archivos
        filename = f"{fecha}-{hora}.jpg"
        path_diario = os.path.join(carpeta, filename)
        path_ultima = os.path.join(REP_BASE, "ultima.jpg")

        # 5. Guardar fotos
        success_diario = cv2.imwrite(path_diario, frame)
        success_ultima = cv2.imwrite(path_ultima, frame)
        
        if success_diario:
            print(f"Imagen guardada: {path_diario}")
        else:
            print(f"Error al guardar la imagen en {path_diario}")

        # Esperar 1 hora (3600 segundos)
        time.sleep(3600)  

except KeyboardInterrupt:
    print("\nCaptura detenida por el usuario.")

# Liberar recursos
cap.release()
cv2.destroyAllWindows()