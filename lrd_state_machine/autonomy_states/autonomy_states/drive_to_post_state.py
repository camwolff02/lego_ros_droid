from yasmin import State
from yasmin.blackboard import Blackboard

import time

class DriveToPostState(State):
    transitions={'success': 'AddWaypoint',
                 'fail': 'DriveToLastWaypoint',
                 'manual': 'ManualDrive'}

    def __init__(self):
        super().__init__(outcomes=[*DriveToPostState.transitions])

    def execute(self, blackboard: Blackboard) -> str:
        print('driving to post')
        time.sleep(1)
        return 'success'
