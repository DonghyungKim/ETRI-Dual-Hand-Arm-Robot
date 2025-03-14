from setuptools import find_packages, setup

package_name = 'sample_etri_dualarm_ctr'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Donghyung Kim',
    maintainer_email='donghyungkim@etri.re.kr',
    description='Package for controlling ETRI\'s dual-arm robot using ROS 2 integrated with IsaacSim',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'sample_sinusoidal_joint_pos_ctrl = sample_etri_dualarm_ctr.sample_sinusoidal_joint_pos_ctrl:main',
                'sample_sequence_joint_pos_ctr = sample_etri_dualarm_ctr.sample_sequence_joint_pos_ctr:main',
                'sample_sinusoidal_arm_vel_ctrl = sample_etri_dualarm_ctr.sample_sinusoidal_arm_vel_ctrl:main'
        ],
    },
)
