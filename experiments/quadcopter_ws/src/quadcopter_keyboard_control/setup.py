from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quadcopter_keyboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py') if os.path.exists('launch') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaleb',
    maintainer_email='kaleb@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_controller = quadcopter_keyboard_control.keyboard_controller:main'
        ],
    },
)
