from setuptools import setup

package_name = 'spot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='maximillian.kirsch@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_teleop_keyboard = spot_teleop.spot_teleop_keyboard:main',
            'spot_teleop_sliders = spot_teleop.spot_teleop_sliders:main'
        ],
    },
)
