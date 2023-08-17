from enum import Enum
from typing import NamedTuple


class GoalType(Enum):
    GNSS_ONLY = 0
    POST = 1
    GATE = 2


class Gps(NamedTuple):
    latitude: float
    longitude: float
