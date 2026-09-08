from dataclasses import dataclass


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
