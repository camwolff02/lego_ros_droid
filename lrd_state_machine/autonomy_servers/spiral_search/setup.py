from setuptools import setup

package_name = 'spiral_search'

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
            f'spiral_search = {package_name}.spiral_search:main',
            f'spiral_search_test = {package_name}.tests.spiral_search_test:main',
            f'state_machine_test = {package_name}.tests.state_machine_test:main',
        ],
    },
)
