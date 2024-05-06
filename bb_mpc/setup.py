from setuptools import find_packages, setup

package_name = 'bb_mpc'

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
    maintainer='emil',
    maintainer_email='emil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = bb_mpc.mpc:main",
            "opti_mpc = bb_mpc.opti_mpc:main",
            "casadi_mpc = bb_mpc.casadi_mpc:main",
            "logger = bb_mpc.logger:main"
        ],
    },
)
