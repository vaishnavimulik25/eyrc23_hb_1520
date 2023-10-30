from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'hb_task_1b'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    #eyrc website
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*'))
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='g10',
    maintainer_email='jitentopiwala7@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # scripts=[
    #     'script/service_node.py',
    #     'script/controller.py',
    #     ],
    entry_points={
        'console_scripts': [
            "service_node = hb_task_1b.service_node:main",
            "controller_node = hb_task_1b.controller:main",
        ],
    },
)
