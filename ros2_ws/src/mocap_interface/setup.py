## ! DO NOT MANUALLY INVOKE THIS setup.py, USE COLCON INSTEAD

from setuptools import setup
from setuptools import find_packages

# fetch values from package.xml
package_name = 'moq_publisher'
setup_args = {
    'name': package_name,
    'packages': find_packages(exclude=['test']),
    'package_dir': {'': 'src'},
    'install_requires': ['setuptools'],
    'zip_safe': True,
    'author': 'Your Name',
    'author_email': 'you@example.com',
    'maintainer': 'Your Name',
    'maintainer_email': 'you@example.com',
    'keywords': ['ROS'],
    'classifiers': [
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
}

setup(**setup_args)