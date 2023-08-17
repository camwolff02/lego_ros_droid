from yasmin import State
from yasmin.blackboard import Blackboard

from .data import Gps

import time

class AddWaypointState(State):
    transitions={'next': 'Idle'}
    counter = 0.0

    def __init__(self):
        super().__init__(outcomes=[*AddWaypointState.transitions])

    def execute(self, blackboard: Blackboard) -> str:
        curr_gps = Gps(self.counter, self.counter)  #TODO get current gps from GPS pub
        self.counter += 1
        print('waypoint added')
        time.sleep(1)

        if 'waypoints' in blackboard:
            blackboard['waypoints'].append(curr_gps)
        else:
            blackboard['waypoints'] = [curr_gps]

        return 'next'
