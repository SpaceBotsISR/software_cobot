from setuptools import find_packages, setup

package_name = 'avatar_overlay'

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
    maintainer='andre-rebelo-teixeira',
    maintainer_email='andre.r.teixeira@tecnico.ulisboa.pt',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_avatar = avatar_overlay.draw_avatar:main'
        ],
    },
)
