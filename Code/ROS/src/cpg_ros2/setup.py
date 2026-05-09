from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cpg_ros2'

def generate_data_files(share_path, dir_name):
    data_files = []
    for root, dirs, files in os.walk(dir_name):
        install_path = os.path.join(share_path, os.path.relpath(root, '.'))
        data_files.append((install_path, [os.path.join(root, f) for f in files]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ] + generate_data_files(os.path.join('share', package_name), 'worlds') + 
        generate_data_files(os.path.join('share', package_name), 'models'),
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='hk08454@st.habib.edu.pk',
    description='Open-loop evolved gait for TQuad',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'cpg_node = cpg_ros2.cpg_node:main',
        ],
    },
)