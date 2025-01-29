from setuptools import find_packages, setup

package_name = 'etri_dualarm_cmd_msg_converter_sim'

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
    description='A command message converter for controlling ETRI\'s dual hand-arm robot in Isaac Sim',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'run_both_arms_pos_ctrl = etri_dualarm_cmd_msg_converter_sim.run_both_arms_pos_ctrl:main',
                'run_both_arms_vel_ctrl = etri_dualarm_cmd_msg_converter_sim.run_both_arms_vel_ctrl:main'
        ],
    },
)
