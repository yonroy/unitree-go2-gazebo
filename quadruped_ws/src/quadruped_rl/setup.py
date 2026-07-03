import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'quadruped_rl'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ONNX policy + config (policy.onnx.data la external data, phai di kem)
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minhtoan',
    maintainer_email='tranminhtoan14062001@gmail.com',
    description='RL locomotion: chay policy ONNX (Go2 velocity-flat) thay gait rule-based',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_policy_node = quadruped_rl.rl_policy_node:main',
        ],
    },
)
