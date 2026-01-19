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
# File  : node_gesture.py
# -*- coding:utf-8 -*-

import math
import time
import random
import queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64


class GestureNode(Node):
    def __init__(self):
        super().__init__('gesture_node')

        # Topic definitions
        self.TOPIC_emotion = "social/emotion"
        self.TOPIC_speech = "speech/to_speak"
        self.TOPIC_pan = "pan_controller/command"
        self.TOPIC_tilt = "tilt_controller/command"
        self.TOPIC_right_arm = "right_arm_controller/command"
        self.TOPIC_left_arm = "left_arm_controller/command"

        # In giro vedo sia "social/gesture" che "/social/gesture" -> ascolto entrambi.
        self.TOPIC_gesture_rel = "social/gesture"
        self.TOPIC_gesture_abs = "/social/gesture"

        # Publisher definitions
        self.emotion_pub = self.create_publisher(String, self.TOPIC_emotion, 10)
        self.speech_pub = self.create_publisher(String, self.TOPIC_speech, 10)
        self.pan_pub = self.create_publisher(Float64, self.TOPIC_pan, 10)
        self.tilt_pub = self.create_publisher(Float64, self.TOPIC_tilt, 10)
        self.left_arm_pub = self.create_publisher(Float64, self.TOPIC_left_arm, 10)
        self.right_arm_pub = self.create_publisher(Float64, self.TOPIC_right_arm, 10)

        # Parameters
        self.declare_parameter("face_reset_timer", 5.0)  # secondi inattività prima di tornare neutro
        self.TIME_DELAY = float(self.get_parameter("face_reset_timer").value)

        # Pose braccia
        self.declare_parameter("arm_neutral_left", 0.0)
        self.declare_parameter("arm_neutral_right", 0.0)
        self.declare_parameter("arm_init_left", 30.0)
        self.declare_parameter("arm_init_right", -30.0)

        # Talk-head tuning (movimento testa mentre parla)
        # Nota: pan_tilt_controller clampa, ma qui teniamo ampiezze piccole per sicurezza
        self.declare_parameter("talk_pan_amp_deg", 10.0)   # oscillazione pan +/- ampiezza
        self.declare_parameter("talk_tilt_amp_deg", 6.0)   # oscillazione tilt +/- ampiezza
        self.declare_parameter("talk_tilt_base_deg", 0.0)  # bias tilt (es. -2 per “guardare leggermente su”)
        self.declare_parameter("talk_pan_jitter_deg", 2.0) # micro-jitter casuale
        self.declare_parameter("talk_tilt_jitter_deg", 2.0)

        # Stato
        self._last_gesture_time = time.monotonic()
        self._talking = False
        self._talk_phase = 0  # per alternare pan/tilt in modo naturale

        # Worker thread: esegue gesture con sleep senza bloccare ROS callbacks
        self._cmd_queue: "queue.Queue[str]" = queue.Queue()
        self._stop_worker = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        # Timer reset
        self.timer = self.create_timer(self.TIME_DELAY, self.reset_gesture)

        # Subscriber
        self.create_subscription(String, self.TOPIC_gesture_rel, self.callback_gesture, 10)
        self.create_subscription(String, self.TOPIC_gesture_abs, self.callback_gesture, 10)

        self.get_logger().info("Gesture node started")

        # posizione iniziale
        self.gesture_zero()

    # -------------------------
    # Publishers helpers
    # -------------------------
    def say(self, msg: str):
        self.get_logger().info(f'speech: {msg}')
        m = String()
        m.data = msg
        self.speech_pub.publish(m)

    def emotion(self, msg: str):
        self.get_logger().info(f'social/emotion: {msg}')
        m = String()
        m.data = msg
        self.emotion_pub.publish(m)

    def pan(self, deg: float):
        m = Float64()
        m.data = float(deg)
        self.pan_pub.publish(m)

    def tilt(self, deg: float):
        m = Float64()
        m.data = float(deg)
        self.tilt_pub.publish(m)

    def left_arm(self, deg: float):
        m = Float64()
        m.data = float(deg)
        self.left_arm_pub.publish(m)

    def right_arm(self, deg: float):
        m = Float64()
        m.data = float(deg)
        self.right_arm_pub.publish(m)

    # -------------------------
    # Head presets
    # -------------------------
    def head_position(self, msg: str):
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
            self.tilt(-20)
        elif msg == 'down':
            self.pan(0)
            self.tilt(20)

    # -------------------------
    # Idle reset
    # -------------------------
    def reset_gesture(self):
        now = time.monotonic()
        idle_s = now - self._last_gesture_time
        if self._talking:
            return
        if idle_s >= self.TIME_DELAY:
            self.get_logger().info("Resetting gesture -> zero (idle timeout)")
            self.gesture_zero()

    # -------------------------
    # Worker thread loop
    # -------------------------
    def _worker_loop(self):
        while rclpy.ok() and not self._stop_worker.is_set():
            try:
                cmd = self._cmd_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                self._execute_gesture(cmd)
            except Exception as e:
                self.get_logger().error(f"Errore esecuzione gesture '{cmd}': {e}")

    def _enqueue(self, gesture: str):
        gesture = (gesture or "").strip()
        if not gesture:
            return
        self._cmd_queue.put(gesture)

    # -------------------------
    # Poses
    # -------------------------
    def gesture_init(self):
        self.head_position('front')
        self.left_arm(float(self.get_parameter("arm_init_left").value))
        self.right_arm(float(self.get_parameter("arm_init_right").value))

    def gesture_zero(self):
        self.head_position('front')
        self.left_arm(float(self.get_parameter("arm_neutral_left").value))
        self.right_arm(float(self.get_parameter("arm_neutral_right").value))

    def gesture_down(self):
        self.head_position('front')
        self.left_arm(-15)
        self.right_arm(15)

    def gesture_up(self):
        self.head_position('front')
        self.left_arm(55)
        self.right_arm(-55)

    # -------------------------
    # Talk helpers (testa + braccia)
    # -------------------------
    def _talk_head_move(self, pan_target: float):
        """
        Muove la testa in modo 'parlato': pan oscillante + tilt leggermente variabile.
        pan_target: tipicamente -amp, 0, +amp
        """
        amp_tilt = float(self.get_parameter("talk_tilt_amp_deg").value)
        base_tilt = float(self.get_parameter("talk_tilt_base_deg").value)
        pan_j = float(self.get_parameter("talk_pan_jitter_deg").value)
        tilt_j = float(self.get_parameter("talk_tilt_jitter_deg").value)

        # piccolo jitter per evitare robot “metronomo”
        pan = float(pan_target) + random.uniform(-pan_j, pan_j)

        # tilt: alterno su/giù leggermente e aggiungo jitter
        # (resta entro valori piccoli; il controller comunque clampa)
        sign = 1.0 if (self._talk_phase % 2 == 0) else -1.0
        tilt = base_tilt + sign * (amp_tilt * 0.5) + random.uniform(-tilt_j, tilt_j)

        self.pan(pan)
        self.tilt(tilt)

        self._talk_phase += 1

    def arms_talk_start(self):
        # stato talk + posa base
        self._talking = True
        self._talk_phase = 0
        self.head_position('front')

        # braccia “pronte”
        self.left_arm(20)
        self.right_arm(-20)

        # un minimo accenno testa
        self._talk_head_move(0.0)

    def arms_talk_stop(self):
        self._talking = False
        self.gesture_zero()

    def arms_talk_1(self):
        # braccia
        self.left_arm(35)
        self.right_arm(10)
        # testa (pan verso dx “guardare interlocutore”)
        amp_pan = float(self.get_parameter("talk_pan_amp_deg").value)
        self._talk_head_move(+amp_pan)

    def arms_talk_2(self):
        self.left_arm(10)
        self.right_arm(35)
        # testa (pan verso sx)
        amp_pan = float(self.get_parameter("talk_pan_amp_deg").value)
        self._talk_head_move(-amp_pan)

    def arms_talk_3(self):
        self.left_arm(30)
        self.right_arm(30)
        # testa (torna al centro)
        self._talk_head_move(0.0)

    # -------------------------
    # Altre gesture demo
    # -------------------------
    def hello(self):
        self.head_position('front')
        self.left_arm(10)
        self.right_arm(-60)
        time.sleep(0.15)
        for _ in range(3):
            self.right_arm(-35)
            time.sleep(0.12)
            self.right_arm(-65)
            time.sleep(0.12)
        self.right_arm(-30)
        self.head_position('front')

    def gesture_indica_sinistra(self):
        self.head_position('left')
        self.left_arm(55)
        self.right_arm(-10)

    def point_left(self):
        self.gesture_indica_sinistra()

    def point_right(self):
        self.head_position('right')
        self.right_arm(-55)
        self.left_arm(10)

    def point_up(self):
        self.head_position('up')
        self.left_arm(45)
        self.right_arm(-45)

    def greeting(self):
        self.gesture_init()
        time.sleep(0.15)
        self.hello()
        time.sleep(0.10)
        self.gesture_init()

    def check_ticket(self):
        self.head_position('down')
        self.left_arm(-20)
        self.right_arm(20)
        time.sleep(0.20)
        self.head_position('front')

    def gesture_anim(self):
        self.gesture_init()
        time.sleep(0.10)
        self.left_arm(45)
        self.right_arm(-15)
        self.pan(10)
        time.sleep(0.10)
        self.left_arm(15)
        self.right_arm(-45)
        self.pan(-10)
        time.sleep(0.10)
        self.gesture_init()
        self.head_position('front')

    # -------------------------
    # Dispatcher
    # -------------------------
    def _execute_gesture(self, gesture: str):
        g = (gesture or "").strip()

        # reset idle timer su ogni gesto
        self._last_gesture_time = time.monotonic()
        try:
            self.timer.reset()
        except Exception:
            pass

        self.get_logger().info(f"Execute gesture: {g}")

        aliases = {
            # talk (alias comodi)
            "talk_start": "arms_talk_start",
            "talk_stop": "arms_talk_stop",
            "talk_1": "arms_talk_1",
            "talk_2": "arms_talk_2",
            "talk_3": "arms_talk_3",

            # vecchio comando generico
            "gesture": "gesture_anim",
        }
        g = aliases.get(g, g)

        if g == 'init':
            self.gesture_init()
        elif g == 'zero':
            self.gesture_zero()
        elif g == 'down':
            self.gesture_down()
        elif g == 'up':
            self.gesture_up()

        elif g == 'hello':
            self.hello()
        elif g == 'indica_sinistra':
            self.gesture_indica_sinistra()
        elif g == 'point_left':
            self.point_left()
        elif g == 'point_right':
            self.point_right()
        elif g == 'point_up':
            self.point_up()
        elif g == 'greeting':
            self.greeting()
        elif g == 'check_ticket':
            self.check_ticket()

        # TALK: braccia + testa
        elif g == 'arms_talk_start':
            self.arms_talk_start()
        elif g == 'arms_talk_stop':
            self.arms_talk_stop()
        elif g == 'arms_talk_1':
            if not self._talking:
                self.arms_talk_start()
            self.arms_talk_1()
        elif g == 'arms_talk_2':
            if not self._talking:
                self.arms_talk_start()
            self.arms_talk_2()
        elif g == 'arms_talk_3':
            if not self._talking:
                self.arms_talk_start()
            self.arms_talk_3()

        elif g == 'gesture_anim':
            self.gesture_anim()
        else:
            self.get_logger().warning(f"Gesture sconosciuta: '{gesture}'")

    # -------------------------
    # ROS callback (non bloccare)
    # -------------------------
    def callback_gesture(self, data: String):
        gesture = (data.data or "").strip()
        if not gesture:
            return
        self.get_logger().info(f"Received gesture: {gesture}")
        self._enqueue(gesture)

    # -------------------------
    # Utils
    # -------------------------
    @staticmethod
    def DEG2RAD(a):
        return a * math.pi / 180.0

    @staticmethod
    def RAD2DEG(a):
        return a / math.pi * 180.0


def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node._stop_worker.set()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
