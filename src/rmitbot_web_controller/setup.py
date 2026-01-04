from setuptools import setup
import os
from glob import glob

package_name = 'rmitbot_web_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'content'), [f for f in glob('content/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'content/css'), glob('content/css/*')),
        (os.path.join('share', package_name, 'content/js'), glob('content/js/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Web Controller for RMIT Bot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = rmitbot_web_controller.web_server:main'
        ],
    },
)
