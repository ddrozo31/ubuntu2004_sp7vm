from setuptools import setup

package_name = 'turtle_move_pkg_g99'

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
    maintainer='ddrozo',
    maintainer_email='ddrozo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # terminal_name = pkg_name.file_name
            'turtle_move_node_g99 = turtle_move_pkg_g99.turtle_move_node_g99:main',
        ],
    },
)
