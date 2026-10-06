from setuptools import find_packages, setup
import glob
package_name = 'rally_robot'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob('resource/*.npy')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']}, zip_safe=True,
    maintainer='정정인', maintainer_email='team@rally.local',
    description='RALLY — 로봇 제어: 스킬 action 서버, 안전 모니터, RG2 그리퍼', license='MIT',
    entry_points={'console_scripts': [
        'legacy_pick_and_place = rally_robot.legacy_pick_and_place:main',   # 선생님 코드 기준선
        'skill_server = rally_robot.skill_server:main',
        'safety_monitor = rally_robot.safety_monitor:main',
        'gripper_node = rally_robot.gripper_node:main',
        'force_publisher = rally_robot.force_publisher:main',
        'calib_sweep = rally_robot.calib_sweep:main',             # 캘리브레이션 자세 순회·촬영
    ]},
)
