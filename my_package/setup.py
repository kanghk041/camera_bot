from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'params'),
            glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kang',
    maintainer_email='kanghk041041@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detection = my_node.yolo_detection:main',
            'path_planning = my_node.path_planning:main',
            'person_follower = my_node.person_follower:main',
        ],
    },
)
