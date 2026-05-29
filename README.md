# 🍓 Cyber-Physical Strawberry Harvesting System

An autonomous harvesting system that integrates collaborative robotics, 
computer vision, and real-time monitoring for strawberry detection, 
counting, and pick-and-place operations.

![Banner](link-a-tu-foto-o-gif-aqui)

---

## 📌 About

This project was developed in collaboration with **CAETEC (Campo Agropecuario 
Experimental del Tec de Monterrey)**. It combines a UR3e cobot, YOLO-based 
fruit detection, and a real-time monitoring architecture to automate 
strawberry harvesting in a controlled agricultural environment.

---

## ⚙️ How it works

1. A camera captures the scene and **YOLO detects and counts strawberries** in real time
2. The system evaluates fruit position and ripeness
3. **Pick and place commands** are sent to the UR3e collaborative robot
4. The robot executes the harvesting movement with the soft robotic gripper
5. All data is sent via **MQTT** and visualized in a **Node-RED dashboard**

---
---

## 🛠️ Tools & Technologies

![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
![YOLO](https://img.shields.io/badge/YOLO-00FFFF?style=for-the-badge&logoColor=black)
![MQTT](https://img.shields.io/badge/MQTT-660066?style=for-the-badge&logo=eclipse-mosquitto&logoColor=white)
![Node-RED](https://img.shields.io/badge/Node--RED-8F0000?style=for-the-badge&logo=nodered&logoColor=white)
![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)

- **Robot:** Universal Robots UR3e
- **Vision:** YOLO object detection
- **Communication:** MQTT protocol
- **Monitoring:** Node-RED dashboard
- **Languages:** Python, C++

---

## 🌱 Context

Developed at **CAETEC**, the experimental agricultural campus of Tecnológico 
de Monterrey in Querétaro, México. The project addresses real challenges in 
agricultural automation by combining soft robotics, AI vision, and 
cyber-physical systems.

---

## 👩‍💻 Author

**Ana Itzel Hernández García**  
B.S. Robotics and Intelligent Systems — Tecnológico de Monterrey  
[LinkedIn](https://www.linkedin.com/in/ana-itzel-hernández-garcía-irs) · 
[GitHub](https://github.com/ItzelHdzz)
