import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3

import socket
import binascii
import time

import struct

GCU_IP = "192.168.144.108"
TCP_PORT = 2332
RECONNECT_DELAY = 5  # seconds

SOME_MIN_LENGTH = 10

sock = None

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('reader')
        self.publisher_ = self.create_publisher(Vector3, 'gimbal_euler', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Vector3()
        data_from_camera = send_null_command()
        extracted_data = struct.unpack("<hhh",data_from_camera[12:18])

        msg.x = extracted_data[0] / 100.0
        msg.y = extracted_data[1] / 100.0
        msg.z = extracted_data[2] / 100.0

        print(extracted_data)
        self.publisher_.publish(msg)
def build_packet(
    order: int,
    param_bytes: bytes = b"",
    roll: int = 0,
    pitch: int = 0,
    yaw: int = 0,
    ctrl_valid: bool = False,
) -> bytes:
    main = bytearray(32)
    main[0:2] = roll.to_bytes(2, "little", signed=True)
    main[2:4] = pitch.to_bytes(2, "little", signed=True)
    main[4:6] = yaw.to_bytes(2, "little", signed=True)
    main[6] = 0x04 if ctrl_valid else 0x00
    main[25] = 0x01

    sub = bytearray(32)

    header = b"\xa8\xe5"
    version = b"\x02"
    order_byte = bytes([order])
    packet = header + b"\x00\x00" + version + main + sub + order_byte + param_bytes

    total_len = len(packet) + 2
    packet = packet[:2] + total_len.to_bytes(2, "little") + packet[4:]

    crc = binascii.crc_hqx(packet, 0)
    full_packet = packet + crc.to_bytes(2, "big")

    # Log full packet in hex for debugging
    # print("SEND:", full_packet.hex(" ").upper())

    return full_packet

def establish_connection_with_handshake():
    global sock
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((GCU_IP, TCP_PORT))
            print("TCP socket connected, sending test command...")

            # Send a null command that the gimbal should respond to
            null_packet = build_packet(order=0x00)
            sock.sendall(null_packet)

            # Attempt to read the response
            response = sock.recv(1024)
            # Here, you'll have to decode the response or check certain bytes
            # For example, if you expect a certain length or header:
            if not response or len(response) < SOME_MIN_LENGTH:
                raise Exception("Handshake failed: no or invalid response")

            # If you get here, you have a valid response -> handshake is successful
            print(f"Connected to GCU at {GCU_IP}:{TCP_PORT} (handshake confirmed)")
            return  # The global sock is valid now

        except Exception as e:
            print(f"Connection/handshake failed: {e}")
            if sock:
                try:
                    sock.close()
                except Exception as close_e:
                    print(f"Error closing socket: {close_e}")
            time.sleep(RECONNECT_DELAY)

def send_null_command():
    global sock
    if sock:
        null_packet = build_packet(order=0x00)
        sock.sendall(null_packet)
        return sock.recv(1024)


def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    establish_connection_with_handshake()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()