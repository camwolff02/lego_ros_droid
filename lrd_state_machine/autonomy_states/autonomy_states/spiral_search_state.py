from yasmin import State
from yasmin.blackboard import Blackboard

from .data import GoalType

from time import time

class SpiralSearchState(State):
    transitions={'post': 'DriveToPost',
                 'gate': 'DriveToGate',
                 'fail': 'DriveToLastWaypoint',
                 'manual': 'ManualDrive'}

    def __init__(self):
        super().__init__(outcomes=[*SpiralSearchState.transitions])

    def execute(self, blackboard: Blackboard) -> str:
        print('spiral searching')
        time.sleep(1)

        if 'goal_type' in blackboard:
            if blackboard['goal_type'] == GoalType.GATE:
                return 'gate'
            elif blackboard['goal_type'] == GoalType.POST:
                return 'post'
        else:
            return 'fail'
