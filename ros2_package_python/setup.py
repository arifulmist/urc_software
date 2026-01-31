from setuptools import find_packages, setup
# 1. Import pybind11 helpers
from pybind11.setup_helpers import Pybind11Extension, build_ext

package_name = 'ros2_package_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # 2. Add the C++ extension module
    ext_modules=[
        Pybind11Extension(
            "teensy_serial_backend", 
            ["ros2_package_python/teensy_serial.cpp"], # Path to your C++ file
        ),
    ],
    # 3. Add the build_ext command
    cmdclass={"build_ext": build_ext},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pybind11'], 
    zip_safe=True,
    maintainer='labiba-ibnat-matin',
    maintainer_email='labiba-ibnat-matin@todo.todo',
    description='ROS 2 package with C++ serial backend',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'my_node = ros2_package_python.my_node:main', # Your Python node
        ],
    },
)
