# robo_deteccion_integrado.py
import numpy as np
from robodk import robolink, robomath
import time
from pruebaCalibFotoFresa import main

# Conexión con RoboDK
RDK = robolink.Robolink()
RDK.setRunMode(robolink.RUNMODE_RUN_ROBOT)  
robot = RDK.Item('UR3e')
fresa = RDK.Item('Fresa')        # opcional: solo para debugging/visual
camara = RDK.Item('CameraFrame')

#Velocidades
robot.setSpeed(speed_linear=600, speed_joints=60)
#Posiciones
home = [90, -90, 90, -90, -90, 0]
foto = [0, -70, 50, -160, 90, 90]
approach = [108.91, 18.15, -110.61, 3.26, 90, -135.25]
salida = [-9.08, -29.85, -123.48, 63.81, 90, -81.5]

#Ir a la posición de foto
robot.MoveJ(foto)
time.sleep(0.5)

#Cargar la matriz generada por el script de visión
T_cam_fresa = main()  # forma (4,4), en metros
print("Matriz (m) cargada desde detección:\n", T_cam_fresa)

# Convertir traslación metros a milímetros para RoboDK
#T_cam_fresa = T_cam_fresa.copy()            # proteger original en memoria
T_cam_fresa[:3, 3] *= 1000.0
print("Matriz convertida a mm:\n", T_cam_fresa)
T_cam_fresa = np.array(T_cam_fresa).reshape(4, 4)

# Convertir a Mat de RoboDK
T_cam_fresa_mat = robomath.Mat(T_cam_fresa.tolist())
print(T_cam_fresa_mat)

camara_flange = camara.Pose()   # Pose() devuelve Mat
flange_robot = robot.Pose()          # Mat del flange respecto base robot



poseFresa = flange_robot * camara_flange * T_cam_fresa_mat
print("poseFresa (resultado):\n", np.array(poseFresa.tolist()))

#lo use para inspeccionar un error:
'''
print("Tipo poseFresa:", type(poseFresa))
print("Contenido poseFresa:", poseFresa)
print("Filas:", len(poseFresa.rows))
if len(poseFresa.rows) > 0:
    print("Columnas en la primera fila:", len(poseFresa.rows[0]))'''

#Obtener solo la posición 
fresa_pos = poseFresa.Pos()  # devuelve lista [x,y,z] en mm
print ("fresa_pos",fresa_pos)
pose_fresaT = robomath.transl(fresa_pos) #solo me interesa la posicion no la orientacion 
print ("pose_fresaT",pose_fresaT)

pick = pose_fresaT*robomath.transl(0,0,-100)*robomath.rotz(robomath.pi/2)*robomath.roty(robomath.pi/7)#*robomath.transl(12,0,10)
prepick = pick*robomath.transl(0,0,-90)

robot.MoveJ(approach)
robot.MoveJ(prepick)
robot.MoveL(pick)



