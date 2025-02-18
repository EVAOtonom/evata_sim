from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'evata_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch dosyalarını ekliyoruz
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='akif',
    maintainer_email='akif@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],  # Replace tests_require
    },
    entry_points={
        'console_scripts': [
            "movement_test =" +package_name +".movement_test:main",
            "jabra_test=" +package_name +".jabra_test:main",
            "odom=" +package_name +".odom:main",
            "zed_test=" +package_name +".zed_test:main",
            "laneDetection=" +package_name +".laneDetection:main"
        ],
    },
)

