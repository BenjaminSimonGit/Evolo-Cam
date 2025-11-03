import pytest
import rclpy
from rclpy.node import Node
from  std_msgs.msg import String
import time


@pytest.fixture(scope='module')
def ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_talker_publishes(ros_init):
    node = rclpy.create_node('test_listener')
    msgs_received = []

    def callback(msg):
        msgs_received.append(msg)

    sub = node.create_subscription(String, 'topic', callback, 10)

    # Ge talker.py lite tid att publicera
    start = time.time()
    while time.time() - start < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if msgs_received:
            break

    node.destroy_subscription(sub)
    node.destroy_node()

    assert len(msgs_received) > 0, "Talker didn't publish any messages!"
