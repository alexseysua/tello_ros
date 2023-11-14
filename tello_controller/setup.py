from setuptools import setup

package_name = 'tello_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johvany Gustave',
    maintainer_email='johvany.gustave@ipsa.fr',
    description='Package containing the visual-based controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "aruco_mission_1.py = tello_controller.aruco_mission_1:main",
            "aruco_mission_2.py = tello_controller.aruco_mission_2:main"
        ],
    },
)
