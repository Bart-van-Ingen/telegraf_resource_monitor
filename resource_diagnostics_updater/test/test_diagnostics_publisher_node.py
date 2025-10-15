from dataclasses import dataclass

import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from resource_diagnostics_updater.diagnosed_resource import DiagnosedResource
from resource_diagnostics_updater.diagnostics_publisher import DiagnosticsPublisher
from resource_diagnostics_updater.resource_diagnostics_updater_node import (
    configure_diagnostics_updaters,
    get_diagnosed_resources_from_config,
)


@pytest.fixture
def test_node():
    rclpy.init()
    test_node = Node("test_node")

    yield test_node

    test_node.destroy_node()
    rclpy.shutdown()


@dataclass
class UpdaterNodeConfigTestParams:
    yaml_str: str
    expected_diagnosed_resources: list[DiagnosedResource]


@pytest.mark.parametrize(
    "test_parameters",
    [
        pytest.param(
            UpdaterNodeConfigTestParams(
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
            UpdaterNodeConfigTestParams(
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
def test_initialize_updaters_from_config(
    test_node: Node,
    test_parameters: UpdaterNodeConfigTestParams,
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

    diagnosed_resources = get_diagnosed_resources_from_config(test_node)

    assert diagnosed_resources == test_parameters.expected_diagnosed_resources

    diagnostic_updater = DiagnosticsPublisher(test_node)

    resource_diagnostic_updaters = configure_diagnostics_updaters(
        diagnosed_resources,
        diagnostic_updater,
    )

    assert len(resource_diagnostic_updaters) == len(test_parameters.expected_diagnosed_resources)

    assert len(diagnostic_updater.all_diagnostic_statuses) == len(
        test_parameters.expected_diagnosed_resources
    )
