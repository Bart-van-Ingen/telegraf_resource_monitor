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

    UnixSocketManager(logger, sensor_message_buffer)
    SensorMessageProcessor(node, sensor_message_buffer)

    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    except (KeyboardInterrupt, RCLError):
        logger.info("system_monitor_node received valid kill signal")

    except Exception as error:
        logger.error(traceback.format_exc())
        raise error


if __name__ == "__main__":
    main()
