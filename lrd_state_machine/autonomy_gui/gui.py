import rclpy
from rclpy.node import Node

import streamlit as st

from sensor_msgs.msg import NavSatFix
from autonomy_interface.msg import Mission


mission_topic_name = '/autonomy/mission'


@st.cache_resource
def create_publisher():
    rclpy.init(args=None)
    return Node('gui_pub').create_publisher(Mission, mission_topic_name, 10)


pub = create_publisher()
mission = Mission()


def run_mission():
    print('running mission')
    mission.manual = False
    pub.publish(mission)


def run_manual_override():
    print('overriding')
    mission.manual = True
    pub.publish(mission)


with st.container():
    st.title('Autonomy Mission Planner')
    st.write('---')

    goal_type = st.radio('Choose goal type', ['GNSS Only', 'Post', 'Gate'])

    match goal_type:
        case 'GNSS Only':
            mission.goal_type = Mission.GNSS_ONLY
        case 'Post':
            mission.goal_type = Mission.POST
        case 'Gate':
            mission.goal_type = Mission.GATE

    num_coords = st.number_input('Number of coordinates', min_value=1)
    left_col, right_col = st.columns(2)
    mission.coordinates = []

    for n in range(num_coords):
        coord = NavSatFix()
        coord.latitude = left_col.number_input(
            'latitude', min_value=-90.0, max_value=90.0, key=f'lat{n}')
        coord.longitude = right_col.number_input(
            'longitude', min_value=-180.0, max_value=180.0, key=f'lon{n}')
        mission.coordinates.append(coord)

    st.write('---')

    left_col, right_col = st.columns(2)
    start_btn = left_col.button('Start Mission', on_click=run_mission)
    manual_btn = right_col.button(
        'Manual Override', on_click=run_manual_override)
