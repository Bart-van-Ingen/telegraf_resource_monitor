import sys
import typing
from dataclasses import dataclass

import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from resource_monitoring_interfaces.msg import Field, Resource


@dataclass
class DiagnosedResource:
    topic: str
    name: str
    field: str
    warning_threshold: float
    error_threshold: float

    def __post_init__(self):
        if self.warning_threshold >= self.error_threshold:
            raise ValueError(
                f"warning threshold {self.warning_threshold} must be less than"
                f" error threshold {self.error_threshold}"
            )


class DiagnosticsPublisherManager:
    """Manages the shared diagnostics publisher and accumulated diagnostic statuses."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.all_diagnostic_statuses: list[DiagnosticStatus] = []

        self.diagnosed_resources = self.get_diagnosed_resources_from_config()

        self.node.get_logger().info(
            "configuring diagnostics publisher for resources:"
            f" {[resource.topic for resource in self.diagnosed_resources]}"
        )

        self.configure_diagnostics_updaters(self.diagnosed_resources)

        self.diagnostics_publisher = node.create_publisher(DiagnosticArray, "/diagnostics", 1)

        self.diagnostics_publisher_timer = node.create_timer(1.0, self.publish_all_diagnostics)

    def configure_diagnostics_updaters(self, diagnosed_resources):
        self.resource_diagnostics_updaters = []

        # Use a reentrant callback group to allow multiple resource callbacks to be processed in
        # parallel without restriction
        callback_group = ReentrantCallbackGroup()

        for resource in diagnosed_resources:
            resource_diagnostics_updater = ResourceDiagnosticsUpdater(
                self,
                resource,
                callback_group,
            )

            self.resource_diagnostics_updaters.append(resource_diagnostics_updater)

    def get_diagnosed_resources_from_config(self) -> list[DiagnosedResource]:
        self.node.declare_parameter('diagnosed_resources', '')
        diagnosed_resources_str = (
            self.node.get_parameter('diagnosed_resources').get_parameter_value().string_value
        )

        diagnosed_resources_dicts = yaml.safe_load(diagnosed_resources_str)

        if not diagnosed_resources_dicts:
            self.node.get_logger().error("No diagnosed resources configured. Exiting.")
            sys.exit(1)

        diagnosed_resources = []

        for resource_dict in diagnosed_resources_dicts:
            self.node.get_logger().debug(f"Loaded diagnosed resource: {resource_dict}")
            diagnosed_resources.append(DiagnosedResource(**resource_dict))

        return diagnosed_resources

    def add_diagnostic_status(self, diagnostic_status: DiagnosticStatus) -> None:
        self.all_diagnostic_statuses.append(diagnostic_status)

    def publish_all_diagnostics(self) -> None:
        self.publish_diagnostics(self.all_diagnostic_statuses)

    def publish_diagnostics(
        self,
        diagnostic_statuses: typing.Union[list[DiagnosticStatus], DiagnosticStatus],
    ) -> None:

        if isinstance(diagnostic_statuses, DiagnosticStatus):
            diagnostic_statuses = [diagnostic_statuses]

        diagnostics_array = DiagnosticArray()
        diagnostics_array.header.stamp = self.node.get_clock().now().to_msg()
        diagnostics_array.status = diagnostic_statuses

        self.node.get_logger().debug(
            f"Publishing diagnostics: {[status.name for status in diagnostic_statuses]}"
        )

        self.diagnostics_publisher.publish(diagnostics_array)


class ResourceDiagnosticsUpdater:

    def __init__(
        self,
        publisher_manager: DiagnosticsPublisherManager,
        diagnosed_resource: DiagnosedResource,
        reentrant_callback_group: ReentrantCallbackGroup,
    ) -> None:

        self.publisher_manager = publisher_manager
        self.diagnosed_resource = diagnosed_resource

        # Create diagnostic status for this resource
        self.diagnostic_status = DiagnosticStatus()
        self.diagnostic_status.name = diagnosed_resource.name
        self.diagnostic_status.message = "unknown"
        self.diagnostic_status.hardware_id = "telegraf_resource_monitor"

        # Register this diagnostic status with the manager
        self.publisher_manager.add_diagnostic_status(self.diagnostic_status)

        self.publisher_manager.node.create_subscription(
            Resource,
            diagnosed_resource.topic,
            self.resource_callback,
            qos_profile_sensor_data,
            callback_group=reentrant_callback_group,
        )

        self.publisher_manager.node.get_logger().info(
            f"subscribed to resource topic '{diagnosed_resource.topic}' for diagnostics."
        )

    def resource_callback(self, resource_message: Resource) -> None:

        diagnosed_resource_field: Field = next(
            field
            for field in resource_message.fields
            if field.name == self.diagnosed_resource.field
        )

        self.diagnostic_status.values = [
            KeyValue(
                key=diagnosed_resource_field.name,
                value=str(diagnosed_resource_field.value),
            )
        ]

        if diagnosed_resource_field.value >= self.diagnosed_resource.error_threshold:
            self.diagnostic_status.level = DiagnosticStatus.ERROR
            self.diagnostic_status.message = (
                f"error for {self.diagnosed_resource.name} ({self.diagnosed_resource.field}):"
                f" {diagnosed_resource_field.value} over error threshold"
                f" {self.diagnosed_resource.error_threshold}"
            )

        elif diagnosed_resource_field.value >= self.diagnosed_resource.warning_threshold:
            self.diagnostic_status.level = DiagnosticStatus.WARN
            self.diagnostic_status.message = (
                f"warning for {self.diagnosed_resource.name} {self.diagnosed_resource.field}:"
                f" {diagnosed_resource_field.value} over warning threshold"
                f" {self.diagnosed_resource.warning_threshold}"
            )

        else:
            self.diagnostic_status.level = DiagnosticStatus.OK
            self.diagnostic_status.message = f"{self.diagnosed_resource.name} ok"
            return

        # only publish if status is warning or error
        self.publish()

    def publish(self) -> None:
        """Publish this resource's diagnostic status."""
        self.publisher_manager.publish_diagnostics(self.diagnostic_status)
