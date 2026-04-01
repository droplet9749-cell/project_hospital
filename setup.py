import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'project_hospital'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # # launch 폴더 안의 파일들을 설치 경로로 복사
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # config 폴더 안의 파일들을 설치 경로로 복사
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',

    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 메인 서비스 실행 명령어
            # ros2 run project_hospital turtlebot_delivery 로 실행할 수 있게 연결
            'turtlebot_delivery = turtle_waffle.turtlebot_delivery:main',
            
            # 주행 좌표 테스트 실행 명령어
            'test_navigation = turtle_waffle.test_navigation:main',
        ],
    },
)
