from setuptools import find_packages, setup

package_name = 'i_guard_camera'

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
    maintainer='mazin',
    maintainer_email='mazin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                        'show_camera = i_guard_camera.show_camera:main',
                        'run_camera = i_guard_camera.run_camera:main',
                        'show_camera_object_detection_real = i_guard_camera.show_camera_object_detection_real:main',
                        'show_camera_object_detection_simulation = i_guard_camera.show_camera_object_detection_simulation:main',

        ],
    },
)
