from setuptools import find_packages, setup
import glob
package_name = 'rally_voice'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob('resource/*.tflite') + glob.glob('resource/*.onnx') + glob.glob('resource/.env')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='정동휘', maintainer_email='team@rally.local',
    description='RALLY — 음성: 웨이크워드, STT, 의도, TTS', license='MIT',
    entry_points={'console_scripts': [
        'get_keyword = rally_voice.get_keyword:main',     # 선생님 코드 기준선 (/get_keyword)
        'intent_node = rally_voice.intent_node:main',
        'tts_node = rally_voice.tts_node:main',
    ]},
)
