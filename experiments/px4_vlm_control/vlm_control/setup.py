from setuptools import find_packages, setup

package_name = 'vlm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaleb',
    maintainer_email='kaleblefko@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vlm_control = vlm_control.vlm_control:main',
            'vlm_control_arrows = vlm_control.vlm_control_arrows:main',
            'vlm_basic_yaw = vlm_control.vlm_basic_yaw:main'
        ],
    },
)
