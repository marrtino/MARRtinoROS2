#!/usr/bin/env python3
# Copyright 2025 robotics-3d.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Ferrarini Fabio
# Email : ferrarini09@gmail.com
# File : face_tracker_controller.py
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

class PIDController:
  def __init__(self, kp, ki, kd):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.prev_error = 0
    self.integral = 0

  def compute(self, error):
    # Calcolo PID
    self.integral += error
    derivative = error - self.prev_error
    output = self.kp * error + self.ki * self.integral + self.kd * derivative
    self.prev_error = error
    return output


class FaceRecognitionAndTrackingNode(Node):
  def __init__(self):
    super().__init__('face_recognition_and_tracking_node')

    # Carica il modello Haar Cascade per il rilevamento facciale (più leggero)
    cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
    self.face_cascade = cv2.CascadeClassifier()
    if not self.face_cascade.load(cascade_path):
      self.get_logger().error(f"Impossibile caricare il classificatore Haar da {cascade_path}")
      rclpy.shutdown()
      return
    self.get_logger().info("Classificatore Haar per volti caricato (versione efficiente).")

    # Rimosso il caricamento del modello Caffe DNN
    # self.net = cv2.dnn.readNetFromCaffe(
    #   'deploy.prototxt',
    #   'res10_300x300_ssd_iter_140000.caffemodel'
    # )

    # Inizializza il CvBridge
    self.bridge = CvBridge()

    # Sottoscrizione al topic della fotocamera
    self.subscription = self.create_subscription(
      Image,
      '/camera/image_raw',
      self.image_callback,
      10
    )
    self.image_face_detection = self.create_publisher(Image, '/face_detector/image_raw', 10)

    # Publisher per il controllo del Dynamixel
    self.dynamixel_control = self.create_publisher(Float64, '/pan_controller/command', 10)
    self.dynamixel_control_tilt = self.create_publisher(Float64, '/tilt_controller/command', 10)

    # Publisher per il numero di volti
    self.face_count_pub = self.create_publisher(Float64, '/nroface', 10)
    # Inizializza i servo a 0 gradi (posizione minima)
    initial_pos = Float64()
    initial_pos.data = 0.0 # 0 gradi

    self.dynamixel_control.publish(initial_pos)    # Pan a 0°
    self.dynamixel_control_tilt.publish(initial_pos) # Tilt a 0°
    # Parametri di default per il servo pan/tilt
    self.servomaxx = 1023 # Massima rotazione servo orizzontale (x)
    self.servomaxy = 1023 # Massima rotazione servo verticale (y)
    self.servomin = 0   # Minima rotazione servo
    self.center_pos_x = 512 # Posizione centrale servo orizzontale (x)
    self.center_pos_y = 512 # Posizione centrale servo verticale (y)
    self.current_pos_x = float(self.center_pos_x)
    self.current_pos_y = float(self.center_pos_y)

    # PID controller per pan e tilt
    self.pid_x = PIDController(0.05, 0.001, 0.01) # Regola questi valori per rallentare il movimento
    self.pid_y = PIDController(0.05, 0.001, 0.01)

    # Calcolo dei margini centrali per il tracciamento
    self.screenmaxx = 640 # Risoluzione massima dello schermo (x)
    self.screenmaxy = 480 # Risoluzione massima dello schermo (y)
    self.center_offset = 100
    self.center_offsety = 60
    self.center_left = (self.screenmaxx / 2) - self.center_offset
    self.center_right = (self.screenmaxx / 2) + self.center_offset
    self.center_up = (self.screenmaxy / 2) - self.center_offsety
    self.center_down = (self.screenmaxy / 2) + self.center_offsety

    # Imposta la posizione iniziale centrale
    self.initial_pose_x = Float64()
    self.initial_pose_x.data = float(self.center_pos_x)
    self.initial_pose_y = Float64()
    self.initial_pose_y.data = float(self.center_pos_y-100)
    self.dynamixel_control.publish(self.initial_pose_x)
    self.dynamixel_control_tilt.publish(self.initial_pose_y)
    self.get_logger().info("Face Tracker Controller v.1.0")
 
  def position_to_degrees(self, position):
    """Converte la posizione Dynamixel (0-1023) in gradi."""
    return position * 300 / 1023

  # La vecchia funzione image_callback (commentata) è stata rimossa per chiarezza.

  def image_callback(self, msg):
    try:
      # Converti il messaggio ROS in un'immagine OpenCV
      frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
      # Capovolgi l'immagine orizzontalmente
      frame = cv2.flip(frame, 1)
    except Exception as e:
      self.get_logger().error(f'Failed to convert image: {e}')
      return

    # Converti in scala di grigi per il classificatore Haar
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = frame.shape[:2]

    # Esegui il rilevamento dei volti (Haar Cascade)
    faces = self.face_cascade.detectMultiScale(
      gray,
      scaleFactor=1.1,
      minNeighbors=5,
      minSize=(40, 40)
    )

    face_count = len(faces)
    face_found = False

    # Se troviamo almeno un volto, tracciamo il primo
    # (Per stabilità, potresti voler tracciare il volto più grande)
    if face_count > 0:
      # Ordina i volti per area (il più grande prima) e prendi quello
      faces = sorted(faces, key=lambda f: f[2]*f[3], reverse=True)
      (x, y, w_face, h_face) = faces[0] # Prendi il volto più grande
      
      startX, startY = x, y
      endX, endY = x + w_face, y + h_face

      # Disegna un rettangolo intorno al volto rilevato
      cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)

      # Calcola il centro del volto rilevato
      face_center_x = (startX + endX) // 2
      face_center_y = (startY + endY) // 2

      # Calcola l'offset rispetto al centro dell'immagine
      offset_x = face_center_x - (w // 2)
      offset_y = face_center_y - (h // 2)

      # Chiama la funzione di tracciamento del volto con gli offset calcolati
      self.track_face(offset_x, offset_y)
      
      # Rimosso il time.sleep(0.1) che era qui
      face_found = True # Indica che un volto è stato trovato

    # Pubblica il numero di volti rilevati
    face_count_msg = Float64()
    face_count_msg.data = float(face_count)
    self.face_count_pub.publish(face_count_msg)

    # Converti l'immagine OpenCV in un messaggio ROS e pubblicala
    try:
      ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
      self.image_face_detection.publish(ros_image_msg)
    except Exception as e:
      self.get_logger().error(f'Failed to publish image: {e}')

  def track_face(self, x, y):
    # Calcolo PID per X e Y
    control_x = self.pid_x.compute(x)
    control_y = self.pid_y.compute(y)

    # Controllo asse X (pan)
    self.current_pos_x += -control_x
    if self.current_pos_x <= self.servomaxx and self.current_pos_x >= self.servomin:
      current_pose_x = Float64()
      current_pose_x.data = 512-self.current_pos_x
      self.dynamixel_control.publish(Float64(data=self.position_to_degrees(current_pose_x.data)))

    # Controllo asse Y (tilt)
    self.current_pos_y += control_y
    if self.current_pos_y <= self.servomaxy and self.current_pos_y >= self.servomin:
      current_pose_y = Float64()
      current_pose_y.data = 512-self.current_pos_y
      self.dynamixel_control_tilt.publish(Float64(data=self.position_to_degrees(current_pose_y.data)))


def main(args=None):
  rclpy.init(args=args)
  face_recognition_and_tracking_node = FaceRecognitionAndTrackingNode()
  rclpy.spin(face_recognition_and_tracking_node)
  face_recognition_and_tracking_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()