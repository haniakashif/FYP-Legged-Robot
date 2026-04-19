import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cheetah_ros2'

# Helper function to recursively capture all files in a directory and preserve their structure
def generate_data_files(share_path, dir_path):
    data_files = []
    for root, dirs, files in os.walk(dir_path):
        install_path = os.path.join(share_path, os.path.relpath(root, '.'))
        file_paths = [os.path.join(root, f) for f in files]
        if file_paths:
            data_files.append((install_path, file_paths))
    return data_files

# Define standard data files
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
]

# Append the nested model directories dynamically
data_files.extend(generate_data_files(os.path.join('share', package_name), 'models'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='hk08454@st.habib.edu.pk',
    description='MIT Cheetah 3 Controller on THex Quadruped',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'effort_controller = cheetah_ros2.effort_controller:main',
            'gait_node = cheetah_ros2.gait_node:main',
            'fsm_node = cheetah_ros2.fsm_node:main',
            'estimator_node = cheetah_ros2.estimator_node:main',
            # 'teleop_node = cheetah_ros2.teleop_node:main',
            'stance_controller = cheetah_ros2.stance_controller_node:main',
            'swing_controller = cheetah_ros2.swing_controller_node:main'
        ],
    },
)