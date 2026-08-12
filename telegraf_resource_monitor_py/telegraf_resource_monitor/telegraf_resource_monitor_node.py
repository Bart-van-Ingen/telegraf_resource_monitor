import contextlib
import traceback

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node

from telegraf_resource_monitor.sensor_message import SensorMessageBuffer
from telegraf_resource_monitor.sensor_message_processor import SensorMessageProcessor
from telegraf_resource_monitor.unix_socket_manager import UnixSocketManager


def main(args=None):
    rclpy.init(args=args)
    node = Node("telegraf_resource_monitor_node")
    logger = node.get_logger()

    sensor_message_buffer = SensorMessageBuffer(logger)

    node.declare_parameter("socket_path", "/tmp/telegraf.sock")
    socket_path = node.get_parameter("socket_path").get_parameter_value().string_value
    unix_socket_manager = UnixSocketManager(logger, sensor_message_buffer, socket_path)
    sensor_message_processor = SensorMessageProcessor(node, sensor_message_buffer)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        logger.info("system_monitor_node received valid kill signal")

    except Exception:
        logger.error(traceback.format_exc())
        raise

    finally:
        sensor_message_processor.shutdown()
        unix_socket_manager.shutdown()
        node.destroy_node()
        with contextlib.suppress(RCLError):
            rclpy.shutdown()


if __name__ == "__main__":
    main()
