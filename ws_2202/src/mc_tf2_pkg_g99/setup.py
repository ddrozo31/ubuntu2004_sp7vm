from setuptools import setup
from glob import glob
import os



package_name = 'mc_tf2_pkg_g99'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py")),
        (os.path.join("share", package_name,'models'), glob("models/*.*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ddrozo',
    maintainer_email='ddrozo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist2odom_g99 = mc_tf2_pkg_g99.twist2odom_g99:main',
        ],
    },
)