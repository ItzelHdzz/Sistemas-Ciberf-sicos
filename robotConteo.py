# controlRobotUR3e.py
import numpy as np
from robodk import robolink, robomath
import time
import serial
from ultralytics import YOLO
from pnpConteo import main
from conteoFresas import contar_fresas

# Modelo YOLO para conteo
detector = YOLO("env\\Lib\\best.pt")

RDK = robolink.Robolink()
RDK.setRunMode(robolink.RUNMODE_RUN_ROBOT)

robot = RDK.Item('UR3e')
fresa = RDK.Item('Fresa')
camara = RDK.Item('CameraFrame')
frameFoto = RDK.Item('Frame 5')
mainFrame = RDK.Item('Setup')

# Velocidades
robot.setSpeed(speed_linear=600, speed_joints=50)

# Posiciones
home = ([90, -90, 90, -90, -90, 0])
foto = ([0, -70, 50, -160, 90, 90])
approach = ([111.93, 21.18, -130.61, 27.23, 69.58, -20])
salida = ([65.92, -32.75, -126.72, 42.37, 44.46, 35.63])
deposito = ([-89.76, -87.73, -145.21, 9.08, 90, 90])

while True:
    robot.MoveJ(foto)
    time.sleep(0.5)

    robot.setPoseFrame(frameFoto)

    # PRIMERA DETECCIÓN Y FOTO
    T_cam_fresa1, foto1 = main()

    # Conteo antes
    num_fresas_antes, _ = contar_fresas(foto1, detector)
    print("Fresas antes de recoger:", num_fresas_antes)

    if T_cam_fresa1 is None:
        if num_fresas_antes == 0:
            print("No quedan fresas. Proceso terminado.")
            break
        else:
            print("Hay fresas pero falló PnP. Reintentando…")
            continue

    print("Matriz cargada desde detección:\n", T_cam_fresa1)

    T_cam_fresa1[:3, 3] *= 1000.0
    T_cam_fresa1 = np.array(T_cam_fresa1).reshape(4, 4)
    T_cam_fresa_mat1 = robomath.Mat(T_cam_fresa1.tolist())

    T_cam_fresa_mat1[2, 3] = 0
    vecT = T_cam_fresa_mat1[0:3, 3]

    current_pose = robot.Pose()
    target_pose = robomath.Mat(current_pose)
    target_pose[0, 3] = vecT[0]
    target_pose[1, 3] = vecT[1]
    target_pose[2, 3] = vecT[2]
    target_pose = target_pose * robomath.transl(80, 0, 0)

    robot.MoveJ(target_pose)
    time.sleep(0.5)

    # SEGUNDA DETECCIÓN
    robot.setPoseFrame(mainFrame)
    T_cam_fresa, foto2 = main()

    T_cam_fresa[:3, 3] *= 1000.0
    T_cam_fresa = np.array(T_cam_fresa).reshape(4, 4)
    T_cam_fresa_mat = robomath.Mat(T_cam_fresa.tolist())

    camara_flange = camara.Pose()
    flange_robot = robot.Pose()

    poseFresa = flange_robot * camara_flange * T_cam_fresa_mat
    fresa_pos = poseFresa.Pos()
    pose_fresaT = robomath.transl(fresa_pos)

    pick = pose_fresaT * robomath.transl(0, 0, -100) * robomath.rotz(robomath.pi/2) * robomath.roty(robomath.pi/7) * robomath.transl(0, 0, 20)
    prepick = pick * robomath.transl(0, 0, -70)
    arranque = pick * robomath.roty(robomath.pi/6) * robomath.transl(15, 0, 0)

    # GRIPPER
    ser = serial.Serial('COM3', 9600, timeout=1)
    time.sleep(2)

    robot.MoveJ(approach)
    robot.MoveJ(prepick)
    robot.MoveL(pick)

    ser.write(b'ON\n')
    print("Gripper activado")

    while True:
        if ser.in_waiting > 0:
            respuesta = ser.readline().decode().strip()
            print("Arduino:", respuesta)
            if "cerrado" in respuesta.lower():
                break
        time.sleep(0.1)

    robot.MoveJ(arranque)
    robot.MoveJ(salida)
    robot.MoveJ(deposito)

    ser.write(b'OFF\n')
    print("Abriendo gripper")
    time.sleep(3)
    ser.close()

    # ===============================
    #   CONTEO DESPUÉS DE RECOGER
    # ===============================
    robot.MoveJ(foto)
    time.sleep(0.5)

    T_after, foto_after = main()
    num_fresas_despues, _ = contar_fresas(foto_after, detector)

    print("Fresas antes:", num_fresas_antes)
    print("Fresas después:", num_fresas_despues)

    if num_fresas_despues == num_fresas_antes - 1:
        print("Recolección exitosa")
    else:
        print("Falló la recolección")


robot.MoveJ(home)
print("Todas las fresas maduras fueron recolectadas.")
