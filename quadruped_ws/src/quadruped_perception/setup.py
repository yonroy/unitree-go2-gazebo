from setuptools import find_packages, setup

package_name = 'quadruped_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minhtoan',
    maintainer_email='tranminhtoan14062001@gmail.com',
    description='YOLO object detector + tracker don + pinhole camera -> target_pose 3D',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = quadruped_perception.object_detector:main',
            'tracker = quadruped_perception.tracker:main',
            'target_pose_node = quadruped_perception.target_pose_node:main',
            'move_target_demo = quadruped_perception.move_target_demo:main',
        ],
    },
)
