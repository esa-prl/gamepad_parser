import os
from glob import glob
from setuptools import setup

package_name = 'gamepad_parser'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miro Voellmy',
    maintainer_email='miro.voellmy@esa.int',
    description='Gamepad parser for PRL rovers',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gamepad_parser_node = gamepad_parser.gamepad_parser:main'
        ],
    },
)
