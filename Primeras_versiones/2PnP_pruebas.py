import numpy as np
from robodk import robolink, robomath
import time
from pruebaCalibFotoFresa import main

RDK = robolink.Robolink()
RDK.setRunMode(robolink.RUNMODE_RUN_ROBOT)  # Modo simulación: .RUNMODE_SIMULATE

robot = RDK.Item('UR3e')
fresa = RDK.Item('Fresa')
camara = RDK.Item('CameraFrame')
frameFoto = RDK.Item('Frame 5')
mainFrame = RDK.Item('Setup')

#Velocidades del robot
robot.setSpeed(speed_linear=600, speed_joints=50)

# Posiciones relevantes
foto = ([0, -70, 50, -160, 90, 90]) #posicione en la que se toma la foto de la fresa
approach =([111.93, 21.18, -130.61, 27.23, 69.58, -20]) #acerca al robot a el setup donde estan las fresas
salida = ([65.92, -32.75, -126.72, 42.37, 44.46, 35.63]) #despues de tomar la fresa, para que salga el robot sin colisionar con el setup
deposito = ([-89.76, -87.73, -145.21, 9.08, 90, 90])

#PRIMER PnP
#Ir a la posición de foto
robot.MoveJ(foto)
time.sleep(0.5)

#Se establece el frame de la camara en la posicion de foto como el frame para movimientos del robot
robot.setPoseFrame(frameFoto)

#Cargar la matriz generada por el script de visión
T_cam_fresa1 = main()  # forma (4,4), en metros
print("Matriz (m) cargada desde detección:\n", T_cam_fresa1)

# Convertir traslación metros a milímetros para RoboDK
#T_cam_fresa = T_cam_fresa.copy()            # proteger original en memoria
T_cam_fresa1[:3, 3] *= 1000.0
print("Matriz convertida a mm:\n", T_cam_fresa1)
T_cam_fresa1 = np.array(T_cam_fresa1).reshape(4, 4)

# Convertir a Mat de RoboDK
T_cam_fresa_mat1 = robomath.Mat(T_cam_fresa1.tolist())

#se "ignora" z, z = 0
T_cam_fresa_mat1[2, 3] = 0        
print("Matriz ignorando Z", T_cam_fresa_mat1)

#Se extrae vector de traslacion
vecT = T_cam_fresa_mat1[0:3,3]
print("Vector de traslacion",vecT)

#Que el robot se quede en su orientacion y solo se mueva en x, y, z
current_pose = robot.Pose()
target_pose = robomath.Mat(current_pose)
target_pose[0,3] = vecT[0]
target_pose[1,3] = vecT[1]
target_pose[2,3] = vecT[2]

target_pose = target_pose*robomath.transl(80,0,0)

#El robot se mueve a la nueva posicion de foto en la que se alinea con la fresa
robot.MoveJ(target_pose)
time.sleep(0.5)

#SEGUNDO PnP
#Se regresa al frame original de movimientos del robot
robot.setPoseFrame(mainFrame)

#Cargar la matriz generada por el script de visión
T_cam_fresa = main()  # forma (4,4), en metros
print("Matriz (m) cargada desde detección:\n", T_cam_fresa)

# Convertir traslación metros a milímetros para RoboDK
T_cam_fresa[:3, 3] *= 1000.0
print("Matriz convertida a mm:\n", T_cam_fresa)
T_cam_fresa = np.array(T_cam_fresa).reshape(4, 4)

# Convertir a Mat de RoboDK
T_cam_fresa_mat = robomath.Mat(T_cam_fresa.tolist())
print(T_cam_fresa_mat)

camara_flange = camara.Pose()   # Camara respecto al flange del robot
flange_robot = robot.Pose()          # Flange respecto base robot

#Transformaciones necesarias para obtener la posicion de la fresa respecto al robot
poseFresa = flange_robot * camara_flange * T_cam_fresa_mat
print("poseFresa (resultado):\n", np.array(poseFresa.tolist()))


#Obtener solo la posición 
fresa_pos = poseFresa.Pos()  # devuelve lista [x,y,z] en mm
print ("fresa_pos",fresa_pos)
pose_fresaT = robomath.transl(fresa_pos) #solo me interesa la posicion no la orientacion 
print ("pose_fresaT",pose_fresaT)

#Posiciones para recolección
pick = pose_fresaT*robomath.transl(0,0,-100)*robomath.rotz(robomath.pi/2)*robomath.roty(robomath.pi/7)*robomath.transl(0,0,20)
prepick = pick*robomath.transl(0,0,-70)
arranque = pick*robomath.roty(robomath.pi/7)*robomath.transl(25,0,0)

#Movimientos del robot
robot.MoveJ(approach)
robot.MoveJ(prepick)
robot.MoveL(pick)
time.sleep(20)
robot.MoveJ(arranque)
robot.MoveJ(salida)
robot.MoveJ(deposito)