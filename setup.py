from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tb3_motion_accuracy_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jvang',
    maintainer_email='johnnyjvang@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'out_and_back_test = tb3_motion_accuracy_test.out_and_back_test:main',
            'square_test = tb3_motion_accuracy_test.square_test:main',
            'circle_test = tb3_motion_accuracy_test.circle_test:main',
            'figure8_test = tb3_motion_accuracy_test.figure8_test:main',
            # Added to print and reset json output
            'reset_results = tb3_motion_accuracy_test.reset_results:main',
            'summary_report = tb3_motion_accuracy_test.summary_report:main',
        ],
    },
)
