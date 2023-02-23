from setuptools import setup
from glob import glob

package_name = 'sixdof'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/urdf',   glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot134',
    maintainer_email='chunfuchen0311@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'float = sixdof.floating_six:main',
            'touch = sixdof.touch_aruco_6:main',
            'hitpile = sixdof.hitpile:main',
            'flip    = sixdof.flip_tile:main',
            'alltask = sixdof.alltask:main',
            'ctrl    = sixdof.controller:main'
        ],
    },
)
