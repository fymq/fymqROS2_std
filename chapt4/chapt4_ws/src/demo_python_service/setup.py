from setuptools import find_packages, setup
from glob import glob #用于launch的通配符

package_name = 'demo_python_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/resource", ['resource/face.jpg','resource/face2.jpg']),#拷贝照片
        ('share/' + package_name+"/launch", glob('launch/*.launch.py')),#拷贝launch文件(通配符寻找任何含.launch.py的文件)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='3281625386@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "learn_face_detect = demo_python_service.learn_face_detect:main",
            "face_detect_node = demo_python_service.face_detect_node:main",
            "face_detect_client_node = demo_python_service.face_detect_client_node:main",
            "turtle_patrol_control = demo_python_service.turtle_patrol_control:main",
            'turtle_patrol_client = demo_python_service.turtle_patrol_client:main',
        ],
    },
)
