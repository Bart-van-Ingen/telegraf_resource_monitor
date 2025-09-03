from dataclasses import dataclass

import pytest
import rclpy
from rclpy.node import Node

from telegraf_resource_monitor.unix_socket_manager import (
    SensorMessageBuffer,
    UnixSocketManager,
)


@pytest.fixture
def test_sensor_message_buffer():
    rclpy.init()
    test_node = Node("test_sensor_message_node")
    logger = test_node.get_logger()
    sensor_message_buffer = SensorMessageBuffer(logger)

    yield sensor_message_buffer

    rclpy.shutdown()


@dataclass
class UnixSocketManagerTestParams:
    decoded_message: str
    expected_number_of_messages: int
    initial_message_buffer: str = ""
    processed_message_buffer: str = ""


@pytest.mark.parametrize(
    "test_parameters",
    [
        pytest.param(
            UnixSocketManagerTestParams(
                decoded_message=(
                    '{"fields":{"free":117752885248,"total":403028406272,'
                    '"used_percent":69.21337618804728},"name":"disk","tags":'
                ),
                expected_number_of_messages=0,
                processed_message_buffer=(
                    '{"fields":{"free":117752885248,"total":403028406272,'
                    '"used_percent":69.21337618804728},"name":"disk","tags":'
                ),
            ),
            id="incomplete_message",
        ),
        pytest.param(
            UnixSocketManagerTestParams(
                decoded_message=(
                    '{"fields":{"free":117752885248,"total":403028406272,'
                    '"used_percent":69.21337618804728},"name":"disk","tags":'
                    '{"path":"root"},"timestamp":1756879911}\n{"fields"'
                    ':{"memory_usage":0.2598787248134613},"name":"procstat","tags":'
                    '{"node_name":"telegraf_resource_monitor"},"timestamp":1756879911}\n'
                ),
                expected_number_of_messages=2,
            ),
            id="2_complete_messages",
        ),
        pytest.param(
            UnixSocketManagerTestParams(
                decoded_message=(
                    '1.206},"name":"sensors","tags":{"chip":"amdgpu-pci-0400","feature":"vddgfx"}'
                    ',"timestamp":1756879912}\n{"fields":{"in_input":0.762},"name":"sensors",'
                    '"tags":{"chip":"amdgpu-pci-0400","feature":"vddnb"},"timestamp":1756879912}'
                    '\n{"fields":{"temp_input":48},"name":"sensors","tags":{"chip":'
                    '"amdgpu-pci-0400","feature":"edge"},"timestamp":1756879912}\n{"fields":'
                    '{"power_average":0.001},"name":"sensors"'
                ),
                initial_message_buffer='{"fields":{"in_input":',
                processed_message_buffer='{"fields":{"power_average":0.001},"name":"sensors"',
                expected_number_of_messages=3,
            ),
            id="3_complete_messages_with_initial_buffer",
        ),
    ],
)
def test_buffer_complete_messages(
    test_sensor_message_buffer: SensorMessageBuffer,
    test_parameters: UnixSocketManagerTestParams,
):
    processed_message_buffer = UnixSocketManager.buffer_complete_messages(
        test_parameters.decoded_message,
        test_parameters.initial_message_buffer,
        test_sensor_message_buffer,
    )

    assert (
        test_sensor_message_buffer.buffer.qsize()
        == test_parameters.expected_number_of_messages
    )
    assert processed_message_buffer == test_parameters.processed_message_buffer
