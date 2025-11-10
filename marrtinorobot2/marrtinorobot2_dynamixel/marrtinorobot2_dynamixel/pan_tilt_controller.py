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
# File  : pantilt_controller.py
#
# corretto errore in fase di esecuzione 03/11/2025
#
#!/usr/bin/env python3


import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from dynamixel_sdk import *  # noqa

try:
    import serial  # pyserial (solo per type/exception hints)
except Exception:  # pragma: no cover
    serial = None


class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')

        # Porta e protocollo
        self.port_handler = PortHandler('/dev/dynamixel')
        self.packet_handler = PacketHandler(1.0)
        self.baudrate = 1_000_000

        # ID motorini
        self.tilt_motor_id = 1
        self.pan_motor_id = 2
        self.right_arm_motor_id = 3
        self.left_arm_motor_id = 4

        # Stato interno
        self.io_lock = threading.Lock()
        self.last_speed = {}  # motor_id -> last speed scritto

        self.init_dynamixel()

        # Topic
        self.pan_subscriber = self.create_subscription(
            Float64, 'pan_controller/command', self.pan_callback, 10)
        self.tilt_subscriber = self.create_subscription(
            Float64, 'tilt_controller/command', self.tilt_callback, 10)
        self.right_arm_subscriber = self.create_subscription(
            Float64, 'right_arm_controller/command', self.right_arm_callback, 10)
        self.left_arm_subscriber = self.create_subscription(
            Float64, 'left_arm_controller/command', self.left_arm_callback, 10)

    # -------------------- Setup porta --------------------

    def init_dynamixel(self):
        if self.port_handler.openPort():
            self.get_logger().info('Port opened successfully.')
        else:
            self.get_logger().error('Failed to open port.')

        if self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().info('Baudrate set successfully.')
        else:
            self.get_logger().error('Failed to set baudrate.')

        # Patch: rendi "flush" a prova di errore (evita crash su tcdrain EIO)
        try:
            ser = self.port_handler.ser
            orig_flush = ser.flush

            def _flush_noexcept():
                try:
                    orig_flush()
                except Exception as e:
                    # Non buttiamo giù il processo per un flush che fallisce
                    self.get_logger().warn(f'Ignored serial flush error: {e}')

            ser.flush = _flush_noexcept  # type: ignore[attr-defined]
            self.get_logger().info('Applied safe flush wrapper to serial port.')
        except Exception as e:
            self.get_logger().warn(f'Could not patch serial flush: {e}')

    def _reopen_port(self) -> bool:
        try:
            self.port_handler.closePort()
        except Exception:
            pass
        time.sleep(0.1)
        ok = self.port_handler.openPort()
        if ok:
            self.port_handler.setBaudRate(self.baudrate)
            self.get_logger().warn('Serial port reopened after error.')
        else:
            self.get_logger().error('Failed to reopen serial port.')
        return ok

    # -------------------- Conversioni --------------------

    @staticmethod
    def degrees_to_position(degrees: float) -> int:
        return int(degrees * 1023.0 / 300.0)

    @staticmethod
    def position_to_degrees(position: int) -> float:
        return position * 300.0 / 1023.0

    # -------------------- Low-level TX helper --------------------

    def _tx_only_write(self, motor_id: int, address: int, value: int, length: int) -> bool:
        """
        Scrive senza attendere risposta. Tenta 2 volte e, se serve, riapre la porta.
        Non rilancia eccezioni: ritorna False in caso di errore, evitando il crash del callback.
        """
        value = int(value)
        with self.io_lock:
            for attempt in range(2):
                try:
                    if length == 1:
                        self.packet_handler.write1ByteTxOnly(self.port_handler, motor_id, address, value)
                    elif length == 2:
                        self.packet_handler.write2ByteTxOnly(self.port_handler, motor_id, address, value)
                    else:
                        raise ValueError(f'Unsupported length={length}')
                    return True
                except Exception as e:
                    self.get_logger().error(
                        f'TX error (id={motor_id}, addr={address}, val={value}, len={length}, '
                        f'attempt={attempt+1}): {e}'
                    )
                    if attempt == 0:
                        self._reopen_port()
                        time.sleep(0.05)
                        continue
                    return False

    # -------------------- Config servo --------------------

    def set_status_return_level(self, motor_id: int, level: int = 0):
        level = max(0, min(2, int(level)))
        if self._tx_only_write(motor_id, 16, level, 1):
            self.get_logger().info(f'Motor {motor_id} status return level set to {level}')

    def set_max_torque(self, motor_id: int, torque_value: int = 1023):
        torque_value = max(0, min(1023, int(torque_value)))
        if self._tx_only_write(motor_id, 14, torque_value, 2):
            self.get_logger().info(f'Motor {motor_id} max torque set to {torque_value}')

    def enable_torque(self, motor_id: int, enable: bool = True):
        if self._tx_only_write(motor_id, 24, 1 if enable else 0, 1):
            self.get_logger().info(f'Motor {motor_id} torque {"enabled" if enable else "disabled"}.')

    # -------------------- Movimento --------------------

    def set_position(self, motor_id: int, position: int, speed: int):
        # Clamp AX 10-bit
        position = max(0, min(1023, int(position)))
        speed = max(1, min(1023, int(speed)))

        # Scrivi la velocità solo se cambia (meno traffico/flush)
        last = self.last_speed.get(motor_id)
        if last != speed:
            if self._tx_only_write(motor_id, 32, speed, 2):
                self.last_speed[motor_id] = speed
                self.get_logger().info(f'Motor {motor_id} speed set to {speed}')
            else:
                # Se fallisce, non proviamo la posizione per non accumulare errori
                return

        if self._tx_only_write(motor_id, 30, position, 2):
            self.get_logger().info(f'Motor {motor_id} moved to position {position}')

    # -------------------- Callback topic (protetti) --------------------

    def pan_callback(self, msg: Float64):
        try:
            self.get_logger().info(f'Received pan command (degrees): {msg.data}')
            degree = float(msg.data)
            position = 512 + self.degrees_to_position(degree)
            pan_position = max(376, min(648, position))
            self.set_position(self.pan_motor_id, pan_position, 50)
        except Exception as e:
            self.get_logger().error(f'PAN callback error: {e}')

    def tilt_callback(self, msg: Float64):
        try:
            self.get_logger().info(f'Received tilt command (degrees): {msg.data}')
            degree = float(msg.data)
            position = 512 - self.degrees_to_position(degree)
            tilt_position = max(410, min(580, position))
            self.set_position(self.tilt_motor_id, tilt_position, 50)
        except Exception as e:
            self.get_logger().error(f'TILT callback error: {e}')

    def right_arm_callback(self, msg: Float64):
        try:
            self.get_logger().info(f'Received right arm command (degrees): {msg.data}')
            deg = max(90.0, min(210.0, 150.0 + float(msg.data)))
            right_arm_position = self.degrees_to_position(deg)
            self.set_position(self.right_arm_motor_id, right_arm_position, 40)
        except Exception as e:
            self.get_logger().error(f'Right arm callback error: {e}')

    def left_arm_callback(self, msg: Float64):
        try:
            self.get_logger().info(f'Received left arm command (degrees): {msg.data}')
            deg = max(90.0, min(210.0, 150.0 - float(msg.data)))
            left_arm_position = self.degrees_to_position(deg)
            self.set_position(self.left_arm_motor_id, left_arm_position, 40)
        except Exception as e:
            self.get_logger().error(f'Left arm callback error: {e}')


# -------------------- main --------------------

def main(args=None):
    rclpy.init(args=args)
    controller = DynamixelController()

    # Config iniziale: niente status su WRITE (SRL=0) per traffico minimo
    for motor_id in (1, 2, 3, 4):
        controller.set_status_return_level(motor_id, 0)
        controller.set_max_torque(motor_id, 1023)
        controller.enable_torque(motor_id, True)
        time.sleep(0.01)

    # Posizioni iniziali
    controller.set_position(1, controller.degrees_to_position(140), 40)  # Tilt
    controller.set_position(2, controller.degrees_to_position(150), 40)  # Pan
    controller.set_position(3, controller.degrees_to_position(150), 40)  # Braccio dx
    controller.set_position(4, controller.degrees_to_position(150), 40)  # Braccio sx

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
