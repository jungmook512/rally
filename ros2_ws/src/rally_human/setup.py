from setuptools import find_packages, setup
package_name = 'rally_human'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='정동휘',
    maintainer_email='team@rally.local',
    description='RALLY — MediaPipe 3D 손 관절, 전신 자세, 제스처',
    license='MIT',
    entry_points={'console_scripts': [
        'hand_pose = rally_human.hand_pose_node:main',
        # 'body_pose = rally_human.body_pose_node:main',   # Day 4 구현 후 등록 (main 없이 등록하면 launch 가 띄우자마자 죽음)
    ]},
)
