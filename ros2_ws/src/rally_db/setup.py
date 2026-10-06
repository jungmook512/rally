from setuptools import find_packages, setup
package_name = 'rally_db'
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
    maintainer='조정묵',
    maintainer_email='team@rally.local',
    description='RALLY — TrialResult·시계열 기록, pgvector 검색, Redis 상태 발행',
    license='MIT',
    entry_points={'console_scripts': [
        'recorder = rally_db.recorder_node:main',
    ]},
)
