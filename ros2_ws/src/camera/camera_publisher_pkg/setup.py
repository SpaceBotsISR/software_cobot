from setuptools import find_packages, setup

package_name = 'camera_publisher_pkg'

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
    maintainer='AndreRTeixeira',
    maintainer_email='andre.r.teixeira@tecnico.ulisboa.pt',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = camera_publisher_pkg.camera_publisher:main'
        ],
    },
)
