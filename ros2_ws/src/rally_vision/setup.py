from setuptools import find_packages, setup
import glob
package_name = 'rally_vision'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob('resource/*.pt') + glob.glob('resource/*.engine') + glob.glob('resource/*.json')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='박성현', maintainer_email='team@rally.local',
    description='RALLY — 비전: 캘리브레이션, YOLO-Seg, 추적·예측', license='MIT',
    entry_points={'console_scripts': [
        'detection = rally_vision.detection:main',             # 선생님 코드 기준선 (/get_3d_position)
        'segmenter = rally_vision.segmenter_node:main',
        'tracker = rally_vision.tracker_node:main',
        'calibration = rally_vision.calibration_node:main',
    ]},
)
