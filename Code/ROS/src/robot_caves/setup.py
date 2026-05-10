from setuptools import setup
import os
from glob import glob

package_name = 'robot_caves'


def generate_data_files(share_path, dir_name):
    data_files = []
    for root, dirs, files in os.walk(dir_name):
        install_path = os.path.join(share_path, os.path.relpath(root, '.'))
        data_files.append((install_path, [os.path.join(root, f) for f in files]))
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files — only the three cave/floor launchers
        (os.path.join('share', package_name, 'launch'), glob('launch/launch_*.py')),

        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # RViz config
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

    # World files (recursive — drop flat_cave.sdf / rough_cave.sdf / rough_floor.sdf here)
    ] + generate_data_files(os.path.join('share', package_name), 'worlds') + [

    # Robot model files (recursive)
    ] + generate_data_files(os.path.join('share', package_name), 'models'),

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='rh08461@st.habib.edu.pk',
    description='Cave and rough-floor launch environments for the legged robot.',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
