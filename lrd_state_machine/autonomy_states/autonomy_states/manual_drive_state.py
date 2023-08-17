from yasmin import State
from yasmin.blackboard import Blackboard

import time

class ManualDriveState(State):
    transitions={'next': 'Idle',
                 'kill': 'Dead'}

    def __init__(self):
        super().__init__(outcomes=[*ManualDriveState.transitions])

    def execute(self, blackboard: Blackboard) -> str:
        print('driving manually')
        time.sleep(1)
        return 'kill'
