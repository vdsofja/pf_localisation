import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pf_localisation'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '**.launch'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '**.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yan',
    maintainer_email='yxw257@student.bham.ac.uk',
    description='The pf_localisation package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'node = pf_localisation.node:main'
            'node = scripts.node:main',
            'twist = scripts.twist:main',
            'listener = scripts.listener:main',
        ],
    }
)
