from rclpy.node import Node
import traceback

import rclpy
from ros_telegraf_monitor.resource_monitor import ResourceMonitor


def main(args=None):
    rclpy.init(args=args)
    system_monitor_node = Node("system_monitoring_node")
    logger = system_monitor_node.get_logger()

    ResourceMonitor(system_monitor_node)

    try:
        rclpy.spin(system_monitor_node)
        system_monitor_node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        logger.info("system_monitor_node received valid kill signal")

    except Exception as error:
        logger.error(traceback.format_exc())
        raise error


if __name__ == "__main__":
    main()
