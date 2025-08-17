from dataclasses import dataclass
import json
from threading import Event
from queue import Queue
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


@dataclass
class SensorMessageBuffer:
    event = Event()
    buffer = Queue()
    logger: RcutilsLogger

    def add_message(self, message: str) -> None:
        self.logger.debug(f"adding message to buffer: {message}")
        sensor_message = SensorMessage.from_json_str(message)
        self.buffer.put(sensor_message)
        self.event.set()  # Notify that a new message is available

    def get_message(self) -> SensorMessage:
        self.logger.debug("getting message in the buffer...")
        return self.buffer.get()

    def is_empty(self) -> bool:
        return self.buffer.empty()
