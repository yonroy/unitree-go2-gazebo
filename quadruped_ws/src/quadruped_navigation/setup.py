from setuptools import find_packages, setup

package_name = 'quadruped_navigation'

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
    description='Action server goto_point: dieu khien robot toi mot toa do (x,y) roi dung',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goto_point_server = quadruped_navigation.goto_point_server:main',
        ],
    },
)
