from setuptools import setup

package_name = 'pandapy_ctrl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'numpy'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'pose_publisher_node = pandapy_ctrl.pose_publisher_node:main',
            'delta_pose_pub_node = pandapy_ctrl.delta_pose_pub_node:main',
            'pose_subscriber_node = pandapy_ctrl.pose_subscriber_node:main',
            'pose_sub_pandapy = pandapy_ctrl.pose_sub_pandapy:main',
            'pose_sub_pandapy_IK = pandapy_ctrl.pose_sub_pandapy_IK:main',
        ],
    },
)
