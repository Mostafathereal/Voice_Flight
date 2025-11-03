import enum
from enum import Enum

class Command(Enum):
    ARM = "hey drone arm"
    DISARM = "hey drone disarm"
    TAKE_OFF = "hey drone take off"
    LAND = "hey drone land"
    HOVER = "hey drone hover"
    MOVE_FORWARD = "hey drone move forward"
    MOVE_BACKWARD = "hey drone move backward"
    MOVE_LEFT = "hey drone move left"
    MOVE_RIGHT = "hey drone move right"