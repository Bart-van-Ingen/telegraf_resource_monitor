import contextlib
import traceback

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from resource_diagnostics_updater.resource_diagnostics_updater import DiagnosticsPublisherManager


def main(args=None):

    rclpy.init(args=args)

    node = Node("resource_diagnostics_updater_node")
    logger = node.get_logger()

    DiagnosticsPublisherManager(node)

    try:
        # Use a MultiThreadedExecutor to allow multiple resource diagnostics updaters to run in
        # parallel
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()

    except KeyboardInterrupt:
        logger.info("system_monitor_node received valid kill signal")

    except Exception as error:
        logger.error(traceback.format_exc())
        raise error

    finally:

        node.destroy_node()

    with contextlib.suppress(RCLError):
        rclpy.shutdown()


if __name__ == "__main__":
    main()
