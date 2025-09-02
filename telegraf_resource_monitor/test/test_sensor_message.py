from dataclasses import dataclass

import pytest
import rclpy
from rclpy.node import Node

from telegraf_resource_monitor.sensor_message import SensorMessage, SensorMessageBuffer


@pytest.fixture
def test_buffer():
    rclpy.init()
    test_node = Node("test_sensor_message_node")

    yield SensorMessageBuffer(test_node.get_logger())

    rclpy.shutdown()


@dataclass
class ProcessorTestParams:
    message: str
    expected_message: SensorMessage


@pytest.mark.parametrize(
    "test_parameters",
    [
        pytest.param(
            ProcessorTestParams(
                message='{"fields":'
                '{"usage_active":10.784313725490524,'
                '"usage_system":1.9607843137252647,'
                '"usage_user":8.823529411764214},'
                '"name":"cpu",'
                '"tags":{"cpu":"cpu3"},'
                '"timestamp":1756666979}',
                expected_message=SensorMessage(
                    name="cpu",
                    tags={"cpu": "cpu3"},
                    fields={
                        "usage_active": 10.784313725490524,
                        "usage_system": 1.9607843137252647,
                        "usage_user": 8.823529411764214,
                    },
                    timestamp=1756666979,
                ),
            ),
            id="cpu_message",
        )
    ],
)
def test_sensor_message_buffer(
    test_buffer: SensorMessageBuffer, test_parameters: ProcessorTestParams
):
    test_buffer.add_message(test_parameters.message)

    assert not test_buffer.is_empty()

    message = test_buffer.get_message()

    assert message == test_parameters.expected_message

    assert test_buffer.is_empty()
