from setuptools import find_packages, setup
package_name = 'rally_brain'
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
    description='RALLY — 상황 요약, 규칙 레이어, LLM advisor, py_trees 행동 트리',
    license='MIT',
    entry_points={'console_scripts': [
        'bt_runner = rally_brain.bt_runner:main',
        'context_builder = rally_brain.context_builder:main',
        'llm_advisor = rally_brain.llm_advisor:main',
    ]},
)
