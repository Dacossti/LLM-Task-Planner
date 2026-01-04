from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'llm_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['llm_nav', 'llm_nav.*']),
    package_data={
        'llm_nav': ['data/*.json'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'nav2_simple_commander',
        'numpy',
        'pyyaml'
    ],
    zip_safe=True,
    maintainer='dacossti',
    maintainer_email='osias.stave@outlook.com',
    description='Navigation through LLM instructions',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_to_pose = llm_nav.nav_to_pose:main',
            'room_detector = llm_nav.room_detector:main'
        ],
    },
)
