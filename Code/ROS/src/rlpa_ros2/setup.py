from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rlpa_ros2'

# Helper function to recursively find all files in a directory
def generate_data_files(share_path, dir_name):
    data_files = []
    for root, dirs, files in os.walk(dir_name):
        # Calculate where to put these files in the install directory
        install_path = os.path.join(share_path, os.path.relpath(root, '.'))
        # Add the files to the list
        data_files.append((install_path, [os.path.join(root, f) for f in files]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Install Launch Files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. Install Config Files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
    # 3. Install World Files (Recursive)
    ] + generate_data_files(os.path.join('share', package_name), 'worlds') + [
        
    # 4. Install Model Files (Recursive)
    ] + generate_data_files(os.path.join('share', package_name), 'models'),

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='rh08461@st.habib.edu.pk',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rl_obs = rlpa_ros2.rl_obs:main',
            'rl_policy = rlpa_ros2.rl_policy:main',
            'rl_action = rlpa_ros2.rl_action:main',
            'flight_recorder = rlpa_ros2.flight_recorder:main',
            'teleop = rlpa_ros2.teleop:main',
        ],
    },
)
