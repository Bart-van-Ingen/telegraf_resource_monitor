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


class DiagnosticsPublisherManager:
    """Manages the shared diagnostics publisher and accumulated diagnostic statuses."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.diagnostic_statuses: list[DiagnosticStatus] = []

        diagnosed_resources = self.get_diagnosed_resources_from_config()

        self.node.get_logger().info(
            "configuring diagnostics publisher for resources:"
            f" {[resource.topic for resource in diagnosed_resources]}"
        )

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

        self.diagnostics_publisher = node.create_publisher(DiagnosticArray, "/diagnostics", 1)

        self.diagnostics_publisher_timer = node.create_timer(1.0, self.publish_all_diagnostics)

    def get_diagnosed_resources_from_config(self) -> list[DiagnosedResource]:
        self.node.declare_parameter('diagnosed_resources', '')
        diagnosed_resources_str = (
            self.node.get_parameter('diagnosed_resources').get_parameter_value().string_value
        )

        diagnosed_resources_dicts = yaml.safe_load(diagnosed_resources_str)

        diagnosed_resources = []
        for resource_dict in diagnosed_resources_dicts:
            self.node.get_logger().debug(f"Loaded diagnosed resource: {resource_dict}")
            diagnosed_resources.append(DiagnosedResource(**resource_dict))

        return diagnosed_resources

    def add_diagnostic_status(self, diagnostic_status: DiagnosticStatus) -> None:
        """Add a diagnostic status to the managed collection."""
        self.diagnostic_statuses.append(diagnostic_status)

    def publish_all_diagnostics(self) -> None:
        """Publish all accumulated diagnostic statuses."""
        self.publish_diagnostics(self.diagnostic_statuses)

    def publish_diagnostics(
        self,
        diagnostic_statuses: typing.Union[list[DiagnosticStatus], DiagnosticStatus],
    ) -> None:
        """Publish specific diagnostic statuses."""
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
    """Creates and manages diagnostic status for a specific resource."""

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
        """Update the diagnostic status for this resource."""
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
