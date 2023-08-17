import os
from glob import glob
from setuptools import setup

package_name = 'detect_tag'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pyx][yma]*'))  # addef for launch file
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='cam',
    maintainer_email='camwolff@email.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'detect_tag = {package_name}.tag_detector:main',
            f'subscribe_tag = {package_name}.tag_subscriber:main',
            f'move_to_tag = {package_name}.move_to_tag:main',
            f'dummy_driver = {package_name}.dummy_driver:main',
            f'dummy_camera = {package_name}.dummy_camera:main',
            f'tag_visualizer = {package_name}.tag_visualizer:main'
        ],
    },
)
