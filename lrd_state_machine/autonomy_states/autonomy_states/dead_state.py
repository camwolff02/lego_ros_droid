from yasmin import State
from yasmin.blackboard import Blackboard

import time

class DeadState(State):
    def __init__(self):
        super().__init__(outcomes=['dead'])

    def execute(self, blackboard: Blackboard) -> str:
        print('we died')
        time.sleep(1)
        return 'dead'