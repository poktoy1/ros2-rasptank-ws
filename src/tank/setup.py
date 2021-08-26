from setuptools import setup

package_name = 'tank'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='poktoy',
    maintainer_email='poktoy1@gmail.com',
    description='ROS2 for tank robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber_twist = tank.subscriber_twist:main',
            'subscriber_arm = tank.subscriber_arm:main',
            'publisher_sonar = tank.publisher_sonar:main',
            'publisher_line_infrared = tank.publisher_line_infrared:main',
            'action_turn_server = tank.action_turn_server:main',
            'action_turn_client = tank.action_turn_client:main',
            'tank_control = tank.tank_control:main',
            'camera_pub = tank.camera_pub:main',
            'camera_sub = tank.camera_sub:main',
        ],
    },
)
