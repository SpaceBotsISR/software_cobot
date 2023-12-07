from dataclasses import dataclass


@dataclass
class OdomPose:
    x: float
    y: float
    theta: float
