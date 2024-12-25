from setuptools import find_packages, setup

package_name = 'controlbot_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haris',
    maintainer_email='haris3112k4@gmail.com',
    description='Python examples',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_parameter=controlbot_py.simple_parameter:main',
            'simple_turtlesim_kinematics=controlbot_py.simple_turtlesim_kinematics:main',
            'simple_tf_kinematics=controlbot_py.simple_tf_kinematics:main',
            'simple_service_server=controlbot_py.simple_service_server:main',
            'simple_service_client=controlbot_py.simple_service_client:main',
            'simple_lifecycle_node=controlbot_py.simple_lifecycle_node:main'
        ],
    },
)
