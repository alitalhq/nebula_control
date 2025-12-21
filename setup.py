from setuptools import find_packages, setup

package_name = 'nebula_control'

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
    maintainer='alitalha',
    maintainer_email='alitqlhq@gmail.com',
    description='ROS2 package for Nebula control nodes',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'operation_manager_node = nebula_control.operation_manager_node:main',
            'gimbal_control_node = hss_gimbal_control.gimbal_control_node:main'
        ],
    },
)
