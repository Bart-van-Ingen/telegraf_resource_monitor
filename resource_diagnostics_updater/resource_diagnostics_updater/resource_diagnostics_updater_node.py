import contextlib
import sys
import traceback

import rclpy
import yaml
from rclpy._rclpy_pybind11 import RCLError
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from resource_diagnostics_updater.diagnosed_resource import DiagnosedResource
from resource_diagnostics_updater.diagnostics_publisher import DiagnosticsPublisher
from resource_diagnostics_updater.resource_diagnostics_updater import ResourceDiagnosticsUpdater


def get_diagnosed_resources_from_config(node: Node) -> list[DiagnosedResource]:

    node.declare_parameter('diagnosed_resources', '')

    diagnosed_resources_yaml = (
        node.get_parameter('diagnosed_resources').get_parameter_value().string_value
    )

    diagnosed_resources_dicts = yaml.safe_load(diagnosed_resources_yaml)

    if not diagnosed_resources_dicts:
        node.get_logger().error("No diagnosed resources configured. Exiting.")
        sys.exit(1)

    diagnosed_resources: list[DiagnosedResource] = []

    for resource_dict in diagnosed_resources_dicts:
        node.get_logger().debug(f"Loaded diagnosed resource: {resource_dict}")
        diagnosed_resources.append(DiagnosedResource(**resource_dict))

    node.get_logger().info(
        "configuring diagnostics publisher for resources:"
        f" {[resource.topic for resource in diagnosed_resources]}"
    )

    return diagnosed_resources


def configure_diagnostics_updaters(
    diagnosed_resources: list[DiagnosedResource],
    diagnostics_publisher: DiagnosticsPublisher,
):
    resource_diagnostics_updaters = []

    # Use a reentrant callback group to allow multiple resource callbacks to be processed in
    # parallel without restriction
    callback_group = ReentrantCallbackGroup()

    for resource in diagnosed_resources:
        resource_diagnostics_updater = ResourceDiagnosticsUpdater(
            diagnostics_publisher=diagnostics_publisher,
            diagnosed_resource=resource,
            reentrant_callback_group=callback_group,
        )

        resource_diagnostics_updaters.append(resource_diagnostics_updater)

    return resource_diagnostics_updaters


def main(args=None):

    rclpy.init(args=args)

    node = Node("resource_diagnostics_updater_node")
    logger = node.get_logger()

    diagnosed_resources = get_diagnosed_resources_from_config(node)

    diagnostics_publisher = DiagnosticsPublisher(node)

    configure_diagnostics_updaters(diagnosed_resources, diagnostics_publisher)

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
