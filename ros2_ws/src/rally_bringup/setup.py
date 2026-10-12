from glob import glob

from setuptools import find_packages, setup
package_name = 'rally_bringup'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='조정묵',
    maintainer_email='team@rally.local',
    description='RALLY — launch 파일, 파라미터(yaml), Lifecycle 기동 순서',
    license='MIT',
    entry_points={'console_scripts': [
        'wait_ready = rally_bringup.wait_ready:main',
        'set_camera_params = rally_bringup.camera_params:main',
    ]},
)
