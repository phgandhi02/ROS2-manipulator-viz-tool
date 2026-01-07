import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from manipulator_viz_core.publisher import MinimalPublisher


def test_MinimalPublisher_is_Node():
    assert issubclass(MinimalPublisher, Node)


def test_MinimalPublisher_has_publisher():
    # setup
    rclpy.init()
    node = MinimalPublisher()

    # nothing to execute

    # assert
    assert hasattr(node, "publisher_")

    # cleanup
    node.destroy_node()
    rclpy.shutdown()


def test_MinimalPublisher_is_msg_sent():
    # Setup
    rclpy.init()
    node = MinimalPublisher()

    # Execute
    msg = node.timer_callback()

    # Assert
    assert msg

    # Cleanup
    rclpy.shutdown()


def test_MinimalPublisher_is_msg_string():
    # Setup
    rclpy.init()
    node = MinimalPublisher()

    # Execute
    msg = node.timer_callback()

    # Assert
    assert isinstance(msg.data, str)

    # Cleanup
    rclpy.shutdown()


def test_MinimalPublisher_if_msg_contains_Hello_World():
    # Setup
    rclpy.init()
    node = MinimalPublisher()

    # Execute
    msg = node.timer_callback()

    # Assert
    assert "Hello World" in msg.data

    # Cleanup
    rclpy.shutdown()
