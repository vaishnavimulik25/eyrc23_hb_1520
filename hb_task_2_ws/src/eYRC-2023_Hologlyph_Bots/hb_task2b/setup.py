from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'hb_task2b'


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
    maintainer='ertslab',
    maintainer_email='srivenkateshwar@e-yantra.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             #'controller = hb_task2b.bot_controller:main',
            'nextgoalpub = hb_task2b.nextGoalPub:main',
            'feedback = hb_task2b.feedback:main',
            #'controller1 = hb_task2b.bot_controller1:main'
            'controller2 = hb_task2b.bot_controller2:main'
            #'controller3 = hb_task2b.bot_controller3:main'
         ],
    },
)
