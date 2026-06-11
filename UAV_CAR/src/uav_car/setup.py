from setuptools import setup
import os
from glob import glob


package_name = 'uav_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processor = uav_car.image_processor:main',
            'qrcode = uav_car.qrcode:main',
            'qrcode2 = uav_car.qrcode2:main',

            # 2023年用到的节点
            'uart = uav_car.uart:main',
            't265 = uav_car.t265:main',
            'navigator = uav_car.navigator:main', 

            'camera = uav_car.camera:main',

            'led = uav_car.led:main',
            'servo = uav_car.servo:main'
        ],
    },
)
