from yasmin import State
from yasmin.blackboard import Blackboard

import time

class DriveToLastWaypointState(State):
    transitions={'success': 'Idle',
                 'fail': 'ManualDrive',
                 'manual': 'ManualDrive'}

    def __init__(self):
        super().__init__(outcomes=[*DriveToLastWaypointState.transitions])

    def execute(self, blackboard: Blackboard) -> str:
        print('driving to last waypoint')
        time.sleep(1)
        return 'success'
