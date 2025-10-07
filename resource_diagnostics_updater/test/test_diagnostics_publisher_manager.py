from dataclasses import dataclass

import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from resource_diagnostics_updater.resource_diagnostics_updater import (
    DiagnosedResource,
    DiagnosticsPublisherManager,
)


@pytest.fixture
def test_node():
    rclpy.init()
    test_node = Node("test_node")

    yield test_node

    test_node.destroy_node()
    rclpy.shutdown()


@dataclass
class PublisherManagerConfigTestParams:
    yaml_str: str
    expected_diagnosed_resources: list[DiagnosedResource]


@pytest.mark.parametrize(
    "test_parameters",
    [
        pytest.param(
            PublisherManagerConfigTestParams(
                yaml_str="""
        - topic: cpu/cpu_total
          name: CPU Usage Active
          field: usage_active
          warning_threshold: 60.0
          error_threshold: 90.0

        - topic: /disk/root
          name: Root Disk Percent Used
          field: used_percent
          warning_threshold: 70.0
          error_threshold: 90.0
        """,
                expected_diagnosed_resources=[
                    DiagnosedResource(
                        topic="cpu/cpu_total",
                        name="CPU Usage Active",
                        field="usage_active",
                        warning_threshold=60.0,
                        error_threshold=90.0,
                    ),
                    DiagnosedResource(
                        topic="/disk/root",
                        name="Root Disk Percent Used",
                        field="used_percent",
                        warning_threshold=70.0,
                        error_threshold=90.0,
                    ),
                ],
            ),
            id="cpu and disk resources",
        ),
        pytest.param(
            PublisherManagerConfigTestParams(
                yaml_str="""
        - topic: /mem
          name: Memory Usage
          field: used_percentage
          warning_threshold: 50.0
          error_threshold: 70.0
        """,
                expected_diagnosed_resources=[
                    DiagnosedResource(
                        topic="/mem",
                        name="Memory Usage",
                        field="used_percentage",
                        warning_threshold=50.0,
                        error_threshold=70.0,
                    ),
                ],
            ),
            id="mem resource",
        ),
    ],
)
def test_diagnostics_publisher_manager_config_loading(
    test_node: Node,
    test_parameters: PublisherManagerConfigTestParams,
):
    diagnosed_resources_parameter = Parameter(
        'diagnosed_resources',
        Parameter.Type.STRING,
        test_parameters.yaml_str,
    )

    test_node.__init__(
        'test_resource_diagnostics_updater_node',
        parameter_overrides=[diagnosed_resources_parameter],
    )

    diagnostics_publisher_manager = DiagnosticsPublisherManager(test_node)

    assert (
        diagnostics_publisher_manager.diagnosed_resources
        == test_parameters.expected_diagnosed_resources
    )

    assert len(diagnostics_publisher_manager.resource_diagnostics_updaters) == len(
        test_parameters.expected_diagnosed_resources
    )

    assert len(diagnostics_publisher_manager.all_diagnostic_statuses) == len(
        test_parameters.expected_diagnosed_resources
    )
