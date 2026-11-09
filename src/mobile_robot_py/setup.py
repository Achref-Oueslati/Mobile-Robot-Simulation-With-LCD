from setuptools import find_packages, setup

package_name = 'mobile_robot_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/rviz_config.rviz']),
        ('share/' + package_name + '/model', ['model/robot.xacro', 'model/robot.gazebo']),
        ('share/' + package_name + '/parameters', ['parameters/bridge_parameters.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='achref_iot',
    maintainer_email='achref_iot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'lcd_display_node = mobile_robot_py.lcd_display_node:main',
        ],
    },
)
