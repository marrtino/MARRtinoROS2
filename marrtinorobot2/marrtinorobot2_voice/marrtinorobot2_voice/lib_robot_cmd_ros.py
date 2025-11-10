#!/usr/bin/env python3

# Copyright 2025 robotics-3d.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Ferrarini Fabio
# Email : ferrarini09@gmail.com
# File  : robot_cmd_ros.py  

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.executors import SingleThreadedExecutor

import threading
import time
from datetime import datetime
import math


class RobotCmdROS(Node):
    def __init__(self):
        super().__init__('robot_cmd_ros')
        # Topic definitions
        self.TOPIC_emotion = "social/emotion"
        self.TOPIC_gesture = "social/gesture"
        self.TOPIC_speech = "speech/to_speak"
        self.TOPIC_language = "speech/language"
        self.TOPIC_ASR = "ASR"
        self.TOPIC_cmdvel = "cmd_vel"
        self.TOPIC_pan = "pan_controller/command"
        self.TOPIC_tilt = "tilt_controller/command"
        self.TOPIC_right_arm = "right_arm_controller/command"
        self.TOPIC_left_arm = "left_arm_controller/command"
        self.TOPIC_right_shoulder_flexion_controller = "right_shoulder_flexion_controller/command"
        self.TOPIC_left_shoulder_flexion_controller = "left_shoulder_flexion_controller/command"
        self.TOPIC_right_shoulder_rotation_controller = "right_shoulder_rotation_controller/command"
        self.TOPIC_left_shoulder_rotation_controller = "left_shoulder_rotation_controller/command"
        self.TOPIC_right_elbow_motor_controller = "right_elbow_controller/command"
        self.TOPIC_left_elbow_motor_controller = "left_elbow_controller/command" 
        self.TOPIC_hand_right_motor_controller = "hand_right_controller/command" 
        self.TOPIC_hand_left_motor_controller = "hand_left_controller/command" 

        self.TOPIC_image = "/camera/image_raw"
        self.TOPIC_getimage = "/getimage"
        self.TOPIC_apriltag = "/apriltag_detections"
        
        # Publisher definitions
        self.robotemotion_pub = self.create_publisher(String, self.TOPIC_emotion, 10)
        self.gesture_pub = self.create_publisher(String, self.TOPIC_gesture, 10)
        self.speech_pub = self.create_publisher(String, self.TOPIC_speech, 10)
        self.language_pub = self.create_publisher(String, self.TOPIC_language, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.TOPIC_cmdvel, 10)
        self.pan_pub = self.create_publisher(Float64, self.TOPIC_pan, 10)
        self.tilt_pub = self.create_publisher(Float64, self.TOPIC_tilt, 10)
        self.right_arm_pub = self.create_publisher(Float64, self.TOPIC_right_arm, 10)
        self.left_arm_pub = self.create_publisher(Float64, self.TOPIC_left_arm, 10)
        self.getimage_pub = self.create_publisher(String, self.TOPIC_getimage, 10)



        self.right_shoulder_flexion_pub = self.create_publisher(Float64, self.TOPIC_right_shoulder_flexion_controller,10)
        self.left_shoulder_flexion_pub = self.create_publisher(Float64, self.TOPIC_left_shoulder_flexion_controller,10)
        self.right_shoulder_rotation_pub = self.create_publisher(Float64, self.TOPIC_right_shoulder_rotation_controller,10)
        self.left_shoulder_rotation_pub = self.create_publisher(Float64, self.TOPIC_left_shoulder_rotation_controller,10)
        
        self.right_elbow_motor_pub = self.create_publisher(Float64, self.TOPIC_right_elbow_motor_controller,10)
        self.left_elbow_motor_pub = self.create_publisher(Float64, self.TOPIC_left_elbow_motor_controller,10)
        self.hand_right_motor_pub = self.create_publisher(Float64, self.TOPIC_hand_right_motor_controller,10)
        self.hand_left_motor_pub = self.create_publisher(Float64, self.TOPIC_hand_left_motor_controller,10)
      
     



        self.get_logger().info('RobotCmdROS v.1.0.1 initialized')
        # inizialize tag
        self.tag_id = -1
        self.tag_size = 0.0
        self.position = None
        self.orientation = None
        self.tag_distance = 0.0
        self.last_tag_id = 0
        self.myasr = ""
        # Define the 'running' attribute to control the thread execution
        self.running = True  # âœ… Added to prevent the AttributeError


   

    

 

    # init function 

    def wait(self,seconds):
        time.sleep(seconds)

    # def tagTrigger():
    #     global tag_trigger_
    #     return tag_trigger_

    def tagClean(self):
        self.tag_id = -1
        time.sleep(1)

    def tagID(self):
        self.get_logger().info(f"ðŸ“Œ Tag ID: {self.tag_id}, Distanza: {self.tag_distance:.2f}m")

        return self.tag_id

    def tagDistance(self):
        self.tag_distance = self.tag_distance
        return self.tag_distance


    def get_user_say(self):
        #self.get_logger().info(f"User Say: {self.myasr}:.2f}m")
        return self.myasr
        
    def clear_asr(self):
        self.myasr = ""

    def stop_threads(self):
        """Ferma i thread Apriltag e ASR in modo ordinato."""
        self.get_logger().info(" Set pippa OFF !!! non Arresto dei thread in corso...")
        # self.running = False  # segnala ai while loop di fermarsi

        # # Controlla ed aspetta che il thread Apriltag termini
        # if hasattr(self, "thread_apriltag") and self.thread_apriltag.is_alive():
        #     self.thread_apriltag.join()
        #     self.get_logger().info("ðŸ“¡ Thread AprilTag fermato.")

        # # Controlla ed aspetta che il thread ASR termini
        # if hasattr(self, "thread_asr") and self.thread_asr.is_alive():
        #     self.thread_asr.join()
        #     self.get_logger().info("ðŸŽ¤ Thread ASR fermato.")

        # self.get_logger().info("âœ… Tutti i thread fermati.")

    
    def begin(self):
        self.get_logger().info('Robot control started')

    def end(self):
        self.stop()          # ferma il movimento
        self.stop_threads()  # ferma i thread ASR + Apriltag
        self.get_logger().info('Robot control stopped')
    
    def stop(self):
        """Stop the robot."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Robot stopped')

    
    def getImage(self):
        self.get_logger().info("Acquiring image...")
        message = String()
        message.data = "getimage" #msg
        self.getimage_pub.publish(message)
        
         
    # movement

    def forward(self, distance):
        """Move forward by the specified distance in meters."""
        self.get_logger().info(f'Moving forward: {distance} meters')
        speed = 0.2  # meters per second
        duration = distance / speed
        twist = Twist()
        twist.linear.x = speed

        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            #self.get_logger().info(f'Publishing: {twist}')
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop()

    def backward(self, distance):
        """Move backward by the specified distance in meters."""
        self.get_logger().info(f'Moving backward: {distance} meters')
        speed = -0.2  # meters per second (negative for backward)
        duration = distance / abs(speed)
        twist = Twist()
        twist.linear.x = speed

        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            #self.get_logger().info(f'Publishing: {twist}')
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop()

    def left(self, angle):
        """Rotate left by the specified angle in degrees."""
        self.get_logger().info(f'Rotating left: {angle} degrees')
        angular_speed = 0.5  # radians per second
        duration = math.radians(angle) / angular_speed
        twist = Twist()
        twist.angular.z = angular_speed

        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            #self.get_logger().info(f'Publishing: {twist}')
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop()

    def right(self, angle):
        """Rotate right by the specified angle in degrees."""
        self.get_logger().info(f'Rotating right: {angle} degrees')
        angular_speed = -0.5  # radians per second (negative for right rotation)
        duration = math.radians(angle) / abs(angular_speed)
        twist = Twist()
        twist.angular.z = angular_speed

        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            #self.get_logger().info(f'Publishing: {twist}')
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop()

    def gesture(self, msg):
        self.get_logger().info(f'gesture: {msg}')
        message = String()
        message.data = msg
        self.gesture_pub.publish(message)
    
    

    def say(self, msg, language):
        self.get_logger().info(f'speech: {msg}')
        self.get_logger().info(f'language: {language}')
        
        # Pubblica la lingua prima
        language_message = String()
        language_message.data = language
        self.language_pub.publish(language_message)
        
        # Aspetta un breve intervallo per garantire l'ordine corretto
        time.sleep(0.1)  # Attendi 100ms
        
        # Poi pubblica il messaggio vocale
        speech_message = String()
        speech_message.data = msg
        self.speech_pub.publish(speech_message)


    def emotion(self, msg):
        self.get_logger().info(f'social/emotion(2): {msg}')
        message = String()
        message.data = msg
        self.robotemotion_pub.publish(message)

    def pan(self, msg):
        self.get_logger().info(f'Pan Position Grade: {msg}')
        message = Float64()
        message.data = float(msg)
        self.pan_pub.publish(message)

    def tilt(self, msg):
        self.get_logger().info(f'Tilt Position Grade: {msg}')
        message = Float64()
        message.data = float(msg)
        self.tilt_pub.publish(message)

    def left_arm(self, msg):
        self.get_logger().info(f'Left Arm Position: {msg}')
        message = Float64()
        message.data = float(msg)
        self.left_arm_pub.publish(message)

    def right_arm(self, msg):
        self.get_logger().info(f'right Arm Position: {msg}')
        message = Float64()
        message.data = float(msg)
        self.right_arm_pub.publish(message)




    def head_position(self, msg):
        self.get_logger().info(f'Head Position: {msg}')
        if msg == 'front':
            self.pan(0)
            self.tilt(0)
        elif msg == 'left':
            self.pan(30)
            self.tilt(0)
        elif msg == 'right':
            self.pan(-30)
            self.tilt(0)
        elif msg == 'up':
            self.pan(0)
            self.tilt(30)
        elif msg == 'down':
            self.pan(0)
            self.tilt(-30)


    def right_shoulder_flexion(self, msg):
        self.get_logger().info(f'Right Shoulder Flexion: {msg}')
        message = Float64()
        message.data = float(msg)
        self.right_shoulder_flexion_pub.publish(message)

    def left_shoulder_flexion(self, msg):
        self.get_logger().info(f'Left Shoulder Flexion: {msg}')
        message = Float64()
        message.data = float(msg)
        self.left_shoulder_flexion_pub.publish(message)

    def right_shoulder_rotation(self, msg):
        self.get_logger().info(f'Right Shoulder Rotation: {msg}')
        message = Float64()
        message.data = float(msg)
        self.right_shoulder_rotation_pub.publish(message)

    def left_shoulder_rotation(self, msg):
        self.get_logger().info(f'Left Shoulder Rotation: {msg}')
        message = Float64()
        message.data = float(msg)
        self.left_shoulder_rotation_pub.publish(message)

    def right_elbow_motor(self, msg):
        self.get_logger().info(f'Right Elbow Motor: {msg}')
        message = Float64()
        message.data = float(msg)
        self.right_elbow_motor_pub.publish(message)

    def left_elbow_motor(self, msg):
        self.get_logger().info(f'Left Elbow Motor: {msg}')
        message = Float64()
        message.data = float(msg)
        self.left_elbow_motor_pub.publish(message)

    def hand_right_motor(self, msg):
        self.get_logger().info(f'Right Hand Motor: {msg}')
        message = Float64()
        message.data = float(msg)
        self.hand_right_motor_pub.publish(message)

    def hand_left_motor(self, msg):
        self.get_logger().info(f'Left Hand Motor: {msg}')
        message = Float64()
        message.data = float(msg)
        self.hand_left_motor_pub.publish(message)
