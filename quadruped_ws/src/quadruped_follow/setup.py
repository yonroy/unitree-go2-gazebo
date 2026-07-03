from setuptools import find_packages, setup

package_name = 'quadruped_follow'

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
    description='Action server track_object: dieu khien robot bam theo muc tieu di dong',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_object_server = quadruped_follow.track_object_server:main',
        ],
    },
)
