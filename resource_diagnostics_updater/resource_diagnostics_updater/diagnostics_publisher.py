from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node


class DiagnosticsPublisher:

    def __init__(self, node: Node) -> None:
        self.node = node
        self.all_diagnostic_statuses: list[DiagnosticStatus] = []

        self.diagnostics_publisher = node.create_publisher(DiagnosticArray, "/diagnostics", 1)

        # Publish diagnostics at 1 Hz as default
        self.diagnostics_publisher_timer = node.create_timer(1.0, self.publish_all_diagnostics)

    def add_diagnostic_status(self, diagnostic_status: DiagnosticStatus) -> None:
        self.all_diagnostic_statuses.append(diagnostic_status)

    def publish_all_diagnostics(self) -> None:
        self.publish_diagnostics(self.all_diagnostic_statuses)

    def publish_diagnostics(
        self,
        diagnostic_statuses: list[DiagnosticStatus] | DiagnosticStatus,
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
