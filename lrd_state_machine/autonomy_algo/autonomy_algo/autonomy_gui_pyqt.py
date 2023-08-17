import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from autonomy_interface.msg import Mission

import sys
from screeninfo import get_monitors

from threading import Thread

from time import time

# import QApplication and all required widgets
from PyQt5.QtWidgets import (
    QApplication,
    QLabel, 
    QWidget, 
    QPushButton, 
    QRadioButton,
    QHBoxLayout,
    QVBoxLayout,
    QMainWindow,
)

from PyQt5.QtGui import QIcon


# class GuiNode(Node):
#     def __init__(self):
#         super().__init__('autonomy_gui')
#         self.mission_pub = self.create_publisher(
#             Mission, 'autonomy/mission', 10)
#         self.mission = Mission()

class AutonomyGui:
    def build_ui(self, main_window):
        """
        Radio buttons:
            - gnss_btn
            - post_btn
            - gate_btn

        Push buttons:
            - run_btn
            - override_btn
        """
        # main GUI window
        screen = get_monitors()[0]
        main_window.setGeometry(screen.width, screen.height, screen.width//2, screen.height)
        main_window.setWindowTitle('Autonomy Mission Planner')

        layout = QVBoxLayout()
        layout.alignment()
        layout.addWidget(QLabel('<h1>Plan Mission</h1>'))

        # set up goal type selection
        self.gnss_btn = QRadioButton('GNSS Only')
        self.post_btn = QRadioButton('Post')
        self.gate_btn = QRadioButton('Gate')
        layout.addWidget(QLabel('Select goal type'))
        layout.addWidget(self.gnss_btn)
        layout.addWidget(self.post_btn)
        layout.addWidget(self.gate_btn)

        # set up run button
        self.run_btn = QPushButton('RUN_MISSION')
        layout.addWidget(self.run_btn)
        
        # set up override button 
        self.override_btn = QPushButton('MANUAL OVERRIDE')
        layout.addWidget(self.override_btn)

        # finish window setup 
        main_window.setLayout(layout)
        


class GuiMainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(GuiMainWindow, self).__init__(parent)
        self.mission = Mission()

        self.mission_topic_name = '/autonomy/mission'
        self.ui = AutonomyGui()
        self.ui.build_ui(main_window=self)
        # self.setWindowIcon(QIcon(...))  # TODO add path to icon
        self.show()

        self.ui.gnss_btn.clicked.connect(self.set_gnss_goal)
        self.ui.post_btn.clicked.connect(self.set_post_goal)
        self.ui.gate_btn.clicked.connect(self.set_gate_goal)
        self.ui.run_btn.clicked.connect(self.run_mission)
        self.ui.override_btn.clicked.connect(self.run_manual_override)

    def set_gnss_goal(self):
        self.mission.goal_type = Mission.GNSS_ONLY

    def set_post_goal(self):
        self.mission.goal_type = Mission.POST
   
    def set_gate_goal(self):
        self.mission.goal_type = Mission.GATE

    def run_mission(self):
        rclpy.init(args=None)
        node = Node('autonomy_gui_node')
        node.create_publisher(Mission,self.mission_topic_name, 10)
        node.publish(self.mission)
        node.destroy_node()
        rclpy.shutdown()

    def run_manual_override(self):
        print('overriding')


def main():
    app = QApplication(sys.argv)
    win = GuiMainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
