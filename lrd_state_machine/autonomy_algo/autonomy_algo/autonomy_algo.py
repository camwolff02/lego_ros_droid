import rclpy

from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

from simple_node import Node
from yasmin.blackboard import Blackboard
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub

from autonomy_states.add_waypoint_state import AddWaypointState
from autonomy_states.dead_state import DeadState
from autonomy_states.drive_between_gates_state import DriveBetweenGateState
from autonomy_states.drive_to_gps_state import DriveToGpsState
from autonomy_states.drive_to_last_waypoint_state import DriveToLastWaypointState
from autonomy_states.drive_to_post_state import DriveToPostState
from autonomy_states.idle_state import IdleState
from autonomy_states.manual_drive_state import ManualDriveState
from autonomy_states.spiral_search_state import SpiralSearchState

from autonomy_interface.action import DriveToGps, SpiralSearch, DriveToMarker


class AutonomyAlgorithm(Node):

    #TODO fix ROS node interfacing
    def __init__(self, gui: bool = False):
        """ Create ROS I/O resources for autonomy state machine """

        super().__init__('autonomy_algo')

        self._gui = gui

        # Blackboard to hold ROS resources
        self._blackboard = Blackboard()

        # Publishers
        self._blackboard['rover_controller'] = \
            self.create_publisher(Twist, '/cmd_vel', 10)        
        self._blackboard['led_controller'] = self.create_publisher(Int32, '/autonomy/led_color', 10)

        # Action Clients
        self._blackboard['drive_to_gps_client'] = \
            self.create_action_client(DriveToGps, '/drive_to_gps', feedback_cb=self._drive_rover)
        self._blackboard['spiral_search_client'] = \
            self.create_action_client(SpiralSearch, '/spiral_search', feedback_cb=self._drive_rover)
        self._blackboard['drive_to_marker_client'] = \
            self.create_action_client(DriveToMarker, '/drive_to_marker', feedback_cb=self._drive_rover)
        
        self._build_state_machine()


    def _build_state_machine(self):
        """Build autonomy state machine (asm)

        s = {'Idle', 'DriveToGps', 'SpiralSearch', 'DecidePostOrGate', 
             'DriveToPost', 'DriveBetweenGate', 'AddWaypoint', 
             'DriveToLastWaypoint', 'ManualDrive', 'Dead'}

        l = {'repeat', 'back', 'next', 'success', 'fail', 'manual', 'post', 
             'gate', 'kill'}
        """
        self.asm = StateMachine(outcomes=['Dead'])

        # S_0, always start by adding starting point to known waypoints
        self.asm.add_state('AddWaypoint', AddWaypointState(),
                transitions=AddWaypointState.transitions)
        self.asm.add_state('Idle', IdleState(), 
                      transitions=IdleState.transitions)
        self.asm.add_state('DriveToGps', DriveToGpsState(), 
                      transitions=DriveToGpsState.transitions)
        self.asm.add_state('SpiralSearch', SpiralSearchState(), 
                      transitions=SpiralSearchState.transitions)
        self.asm.add_state('DriveToPost', DriveToPostState(),
                      transitions=DriveToPostState.transitions)
        self.asm.add_state('DriveBetweenGate', DriveBetweenGateState(),
                      transitions=DriveBetweenGateState.transitions)
        self.asm.add_state('DriveToLastWaypoint', DriveToLastWaypointState(),
                      transitions=DriveToLastWaypointState.transitions)
        self.asm.add_state('ManualDrive', ManualDriveState(),
                      transitions=ManualDriveState.transitions)
        self.asm.add_state('Dead', DeadState())

        if self._gui:
            print('starting gui')
            YasminViewerPub(self, 'AUTONOMY_STATE_MACHINE', self.asm)
        
        print(self.asm(self._blackboard))  # execute


    def _drive_rover(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Received feedback: Linear: {feedback.cmd_vel.linear.x}, \
                Angular: {feedback.cmd_vel.angular.z}');
    
        self.self._blackboard['rover_controller'].publish(feedback.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomyAlgorithm(gui=True)
    node.join_spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
