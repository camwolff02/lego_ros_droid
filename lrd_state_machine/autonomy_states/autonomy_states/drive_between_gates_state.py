from yasmin import State
from yasmin.blackboard import Blackboard

import time

class DriveBetweenGateState(State):
    transitions={'success': 'AddWaypoint',
                 'fail': 'DriveToLastWaypoint',
                 'manual': 'ManualDrive'}

    def __init__(self):
        super().__init__(outcomes=[*DriveBetweenGateState.transitions])

    def execute(self, blackboard: Blackboard) -> str:
        print('driving between gates')
        time.sleep(1)
        return 'success'
