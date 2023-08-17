from setuptools import setup

package_name = 'drive_to_marker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='camwolff',
    maintainer_email='36940948+camwolff02@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'drive_to_marker = {package_name}.drive_to_marker:main',
            f'drive_to_marker_test = {package_name}.tests.drive_to_marker_test:main',
            f'state_machine_test = {package_name}.tests.state_machine_test:main',
        ],
    },
)
