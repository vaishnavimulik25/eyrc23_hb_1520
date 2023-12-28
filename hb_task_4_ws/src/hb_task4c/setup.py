from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'hb_task4c'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='g10',
    maintainer_email='jitentopiwala7@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'feedback = hb_task4c.feedback:main',
                    'nextgoalpub = hb_task2b.nextGoalPub:main',
                    'controller1 = hb_task4c.bot_controller1:main',
                    'controller2 = hb_task4c.bot_controller2:main',
                    'controller3 = hb_task4c.bot_controller3:main'
        ],
    },
)
