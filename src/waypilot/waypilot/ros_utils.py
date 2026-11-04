"""Small helpers shared by the WayPilot nodes."""

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

_DYNAMIC = ParameterDescriptor(dynamic_typing=True)


def declare_float(node: Node, name: str, default: float) -> float:
    """
    Declare a numeric parameter and return it as float.

    Uses dynamic typing so YAML values written as integers (``timeout: 0``)
    are accepted instead of failing with InvalidParameterTypeException.
    """
    return float(node.declare_parameter(name, float(default), _DYNAMIC).value)


def declare_int(node: Node, name: str, default: int) -> int:
    return int(node.declare_parameter(name, int(default), _DYNAMIC).value)
