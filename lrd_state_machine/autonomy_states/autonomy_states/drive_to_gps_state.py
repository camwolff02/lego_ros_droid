import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from yasmin import State
from yasmin.blackboard import Blackboard

from autonomy_interface.action import DriveToGps

from .data import GoalType

import time

class DriveToGpsState(State):
    transitions={'idle': 'DriveToGps',
                 'next': 'SpiralSearch',
                 'success': 'AddWaypoint',
                 'fail': 'DriveToLastWaypoint',
                 'manual': 'ManualDrive'}

    def __init__(self):
        super().__init__(outcomes=[*DriveToGpsState.transitions])
        # self.drive_to_gps_client = DriveToGpsClient()

    def execute(self, blackboard: Blackboard) -> str:
        # try:
        #     rclpy.init(args=None)
        #     print('[DRIVE_TO_GPS] starting automated navigation')
        #     self.drive_to_gps_client.send_goal(blackboard)
        #     rclpy.spin(self.drive_to_gps_client)
        #     # Once client is finished, return results
        #     return blackboard['drive_to_gps_goal_result'] 
        # except KeyboardInterrupt:
        #     rclpy.shutdown()
        #     print('\n[OVERRIDE] switching to manual')
        #     return 'manual'
        
        print('driving to gps')
        time.sleep(1)
        if 'goal_type' in blackboard:
            if blackboard['goal_type'] == GoalType.GNSS_ONLY:
                return 'success'
            else:
                return 'next'

        return 'fail'


# class DriveToGpsClient(Node):
#     def __init__(self):
#         super().__init__('drive_to_gps_client')
#         self._action_client = ActionClient(self, DriveToGps, 'drive_to_gps')
    
#     def send_goal(self, blackboard: Blackboard) -> str:
#         self.blackboard = blackboard

#         # Set goal to requested GPS coordinate
#         goal_msg = DriveToGps.Goal()
#         gps = NavSatFix()
#         gps.latitude, gps.longitude = blackboard['target_gps']
#         goal_msg.c = gps

#         self._action_client.wait_for_server()

#         # send goal to server and define callback if goal is accepted
#         self._send_goal_future = self._action_client.send_goal_async(
#             goal_msg)
        
#         self._send_goal_future.add_done_callback(self._goal_response_callback)


#     def _goal_response_callback(self, future):
#         """Called when server accepts or rejects goal.

#         Args:
#             future.result() (bool): true if goal accepted, false otherwise.
#         """
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             print('[ERROR] Goal rejected, reverting to idle')
#             self.blackboard['drive_to_gps_result'] = 'idle'
#             return

#         print('[SUCCESS] Goal accepted')

#         # ask server for result of action and define callback one result is reached
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self._get_result_callback)

#     def _get_result_callback(self, future):
#         """Called when server completes goal.
        
#         Args:
#             future.result().result: object holding result
#         """
#         result = future.result().result
        
#         # now that we have result, decide outcome of state
#         if not result.success:
#             print(f'[ERROR] Goal failed, driving to last waypoint')
#             self.blackboard['drive_to_gps_result'] = 'fail'
        
#         elif self.blackboard['goal_type'] == GoalType.GNSS_ONLY:
#             self.blackboard['drive_to_gps_result'] = 'success'

#         else:
#             self.blackboard['drive_to_gps_result'] = 'next'

#         print(f'[SUCCESS] Goal reached, moving to next state')
#         rclpy.shutdown()
