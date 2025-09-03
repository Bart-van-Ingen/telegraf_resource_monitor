import traceback

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node

from telegraf_resource_monitor.sensor_message import SensorMessageBuffer
from telegraf_resource_monitor.sensor_message_processor import SensorMessageProcessor
from telegraf_resource_monitor.unix_socket_manager import UnixSocketManager
import contextlib


def main(args=None):
    rclpy.init(args=args)
    node = Node("telegraf_resource_monitor_node")
    logger = node.get_logger()

    sensor_message_buffer = SensorMessageBuffer(logger)

    unix_socket_manager = UnixSocketManager(logger, sensor_message_buffer)
    sensor_message_processor = SensorMessageProcessor(node, sensor_message_buffer)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        logger.info("system_monitor_node received valid kill signal")

    except Exception as error:
        logger.error(traceback.format_exc())
        raise error

    finally:
        sensor_message_processor.shutdown()
        unix_socket_manager.shutdown()
        node.destroy_node()

    with contextlib.suppress(RCLError):
        rclpy.shutdown()


if __name__ == "__main__":
    main()
