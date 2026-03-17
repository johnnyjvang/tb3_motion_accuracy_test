from setuptools import find_packages, setup

package_name = 'tb3_motion_accuracy_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'reset_results = tb3_motion_accuracy_test.reset_results:main',
            'out_and_back_test = tb3_motion_accuracy_test.out_and_back_test:main',
            'square_test = tb3_motion_accuracy_test.square_test:main',
        ],
    },
)
