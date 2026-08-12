import json
from dataclasses import dataclass
from queue import Empty, Queue

from rclpy.impl.rcutils_logger import RcutilsLogger


@dataclass
class SensorMessage:
    name: str
    tags: dict
    fields: dict
    timestamp: int

    @staticmethod
    def from_json_str(json_str: str) -> "SensorMessage":
        sensor_dict = json.loads(json_str)
        return SensorMessage(**sensor_dict)


class SensorMessageBuffer:
    def __init__(self, logger: RcutilsLogger) -> None:
        self.logger = logger
        self.buffer: Queue[SensorMessage] = Queue()

    def add_message(self, message: str) -> None:
        self.logger.debug(f"adding message to buffer: {message}")
        sensor_message = SensorMessage.from_json_str(message)
        self.buffer.put(sensor_message)

    def get_message(self, timeout: float = 0.1) -> SensorMessage | None:
        self.logger.debug("getting message from buffer...")
        try:
            return self.buffer.get(block=True, timeout=timeout)
        except Empty:
            return None

    def is_empty(self) -> bool:
        return self.buffer.empty()
